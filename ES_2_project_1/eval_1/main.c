#include "TM4C123.h"
#include <stdint.h>
#include <stdio.h>

/* =======================================================
   ---------- PE1 (Motor Pulse Output) ----------
   ======================================================= */
#define GPIO_PORTE_CLOCK_EN   0x10
#define GPIO_PORTE_PIN1       0x02

/* =======================================================
   ---------- Timer Clock Enables ----------
   ======================================================= */
#define TIMER1_CLOCK_EN       (1 << 1)
#define TIMER2_CLOCK_EN       (1 << 2)

/* =======================================================
   ---------- Function Prototypes ----------
   ======================================================= */
void PLL_Init(void);
void PE1_Digital_Output_Init(void);
void ADC0_Init(void);
uint16_t ADC0_Read(void);
void Timer1A_Init_Periodic_ms(uint32_t period_ms);
void Timer2A_Init_OneShot_us(uint32_t delay_us);
void UART0_Init(void);
void UART0_TxChar(char c);
void UART0_TxString(char *str);

/* ---------- ISR Prototypes ---------- */
void TIMER1A_Handler(void);
void TIMER2A_Handler(void);

/* ---------- Globals ---------- */
volatile uint32_t pulse_width_us = 1000;
volatile uint32_t rpm = 0;
unsigned volatile long j;

/* =======================================================
   ---------- MAIN FUNCTION ----------
   ======================================================= */
int main(void) {
    PLL_Init();
    PE1_Digital_Output_Init();
    ADC0_Init();
    UART0_Init();

    Timer2A_Init_OneShot_us(pulse_width_us);
    Timer1A_Init_Periodic_ms(20U);

    while (1) {
        // Everything handled in interrupts
    }
}

/* =======================================================
   ---------- UART0 INITIALIZATION ----------
   ======================================================= */
void UART0_Init(void) {
    SYSCTL->RCGCUART |= 0x01;   // Enable UART0 clock
    SYSCTL->RCGCGPIO |= 0x01;   // Enable Port A clock
    for (j = 0; j < 3; j++);

    GPIOA->AFSEL |= 0x03;       // Enable alt function for PA0, PA1
    GPIOA->DEN   |= 0x03;       // Digital enable PA0, PA1
    GPIOA->PCTL  = (GPIOA->PCTL & 0xFFFFFF00) | 0x00000011; // UART0 pins
    GPIOA->AMSEL &= ~0x03;      // Disable analog

    UART0->CTL &= ~0x01;        // Disable UART for config
    UART0->IBRD = 104;          // 16 MHz / (16*9600) = 104.166
    UART0->FBRD = 11;           // Fraction part -> 0.166*64+0.5=11
    UART0->LCRH = 0x70;         // 8-bit, no parity, 1 stop
    UART0->CC = 0x0;            // Use system clock
    UART0->CTL = 0x301;         // Enable UART, TXE, RXE
}

void UART0_TxChar(char c) {
    while ((UART0->FR & 0x20) != 0); // Wait until TXFF is 0
    UART0->DR = c;
}

void UART0_TxString(char *str) {
    while (*str) UART0_TxChar(*str++);
}

/* =======================================================
   ---------- GPIO Configuration ----------
   ======================================================= */
void PE1_Digital_Output_Init(void) {
    SYSCTL->RCGCGPIO |= GPIO_PORTE_CLOCK_EN;
    for (j = 0; j < 3; j++);

    GPIOE->AFSEL &= ~GPIO_PORTE_PIN1;
    GPIOE->DEN   |= GPIO_PORTE_PIN1;
    GPIOE->DIR   |= GPIO_PORTE_PIN1;
    GPIOE->DATA  &= ~GPIO_PORTE_PIN1;
}

/* =======================================================
   ---------- ADC0 Initialization ----------
   ======================================================= */
void ADC0_Init(void) {
    SYSCTL->RCGCADC |= 0x01;
    SYSCTL->RCGCGPIO |= 0x10;
    for (j = 0; j < 3; j++);

    GPIOE->PDR |= 0x08;
    GPIOE->AFSEL |= 0x08;
    GPIOE->DEN &= ~0x08;
    GPIOE->AMSEL |= 0x08;

    ADC0->ACTSS &= ~0x08;
    ADC0->EMUX &= ~0xF000;
    ADC0->SSMUX3 = 0;
    ADC0->SSCTL3 = 0x06;
    ADC0->ACTSS |= 0x08;
}

uint16_t ADC0_Read(void) {
    ADC0->PSSI = 0x08;
    while ((ADC0->RIS & 0x08) == 0);
    uint16_t result = ADC0->SSFIFO3 & 0xFFF;
    ADC0->ISC = 0x08;
    return result;
}

/* =======================================================
   ---------- Timer2A One-Shot ----------
   ======================================================= */
void Timer2A_Init_OneShot_us(uint32_t delay_us) {
    SYSCTL->RCGCTIMER |= TIMER2_CLOCK_EN;
    for (j = 0; j < 3; j++);

    TIMER2->CTL = 0x00;
    TIMER2->CFG = 0x04;
    TIMER2->TAMR = 0x01;
    TIMER2->TAPR = 16 - 1;
    TIMER2->TAILR = delay_us - 1;
    TIMER2->ICR = 0x01;
    TIMER2->IMR |= 0x01;

    NVIC_SetPriority(TIMER2A_IRQn, 2);
    NVIC_EnableIRQ(TIMER2A_IRQn);
}

/* =======================================================
   ---------- Timer1A Periodic ----------
   ======================================================= */
void Timer1A_Init_Periodic_ms(uint32_t period_ms) {
    SYSCTL->RCGCTIMER |= TIMER1_CLOCK_EN;
    for (j = 0; j < 3; j++);

    TIMER1->CTL = 0x00;
    TIMER1->CFG = 0x04;
    TIMER1->TAMR = 0x02;
    TIMER1->TAPR = 250 - 1;
    TIMER1->TAILR = (64 * period_ms) - 1;
    TIMER1->ICR = 0x01;
    TIMER1->IMR |= 0x01;

    NVIC_SetPriority(TIMER1A_IRQn, 1);
    NVIC_EnableIRQ(TIMER1A_IRQn);

    TIMER1->CTL |= 0x01;
}

/* =======================================================
   ---------- INTERRUPT HANDLERS ----------
   ======================================================= */
void TIMER1A_Handler(void) {
    static char buffer[32];
    uint16_t adc_value = ADC0_Read();

    // Map ADC -> Pulse width (1000–2000 µs)
    pulse_width_us = 1000 + (adc_value * 1000) / 4095;

    // Convert to RPM (example: assume 1000–2000 µs = 0–5000 RPM)
    rpm = (pulse_width_us - 1000) * 5;

    // Update pulse timer
    TIMER2->TAILR = pulse_width_us - 1;
    GPIOE->DATA |= GPIO_PORTE_PIN1;
    TIMER2->CTL |= 0x01;

    // Send RPM to PC
    sprintf(buffer, "RPM:%lu\n", rpm);
    UART0_TxString(buffer);

    TIMER1->ICR = 0x01;
}

void TIMER2A_Handler(void) {
    TIMER2->ICR = 0x01;
    GPIOE->DATA &= ~GPIO_PORTE_PIN1;
    TIMER2->CTL &= ~0x01;
}

/* =======================================================
   ---------- PLL ----------
   ======================================================= */
void PLL_Init(void) {
    SYSCTL->RCC2 |= 0x80000000;
    SYSCTL->RCC2 |= 0x00000800;
    SYSCTL->RCC = (SYSCTL->RCC & ~0x000007C0) + 0x00000540;
    SYSCTL->RCC2 &= ~0x00000070;
    SYSCTL->RCC2 &= ~0x00002000;
    SYSCTL->RCC2 |= 0x40000000;
    SYSCTL->RCC2 = (SYSCTL->RCC2 & ~0x1FC00000) + (24 << 22);
    while ((SYSCTL->RIS & 0x00000040) == 0);
    SYSCTL->RCC2 &= ~0x00000800;
}