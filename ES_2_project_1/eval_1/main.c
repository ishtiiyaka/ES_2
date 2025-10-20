#include "TM4C123.h"
#include <stdint.h>
#include <stdio.h>

/* ---------- Pin Definitions ---------- */
#define GPIO_PORTE_CLOCK_EN  0x10
#define GPIO_PORTB_CLOCK_EN  0x02
#define GPIO_PORTE_PIN1      0x02    // Motor PWM output (PE1)
#define GPIO_PORTE_PIN3      0x08    // Potentiometer input (PE3)
#define GPIO_PORTB_PIN1      0x02    // Optocoupler input (PB1)

/* ---------- Timer Clock Enables ---------- */
#define TIMER1_CLOCK_EN      (1 << 1)
#define TIMER2_CLOCK_EN      (1 << 2)
#define TIMER3_CLOCK_EN      (1 << 3)

/* ---------- Globals ---------- */
volatile uint32_t pulse_width_us = 1000;
volatile uint32_t rpm = 0;
volatile uint32_t pulse_count = 0;
unsigned volatile long j;

/* ---------- Function Prototypes ---------- */
void PLL_Init(void);
void PE1_Digital_Output_Init(void);
void PE3_ADC_Init(void);
uint16_t ADC0_Read(void);
void PB1_Optocoupler_Init(void);
void Timer1A_Init_Periodic_ms(uint32_t period_ms);
void Timer2A_Init_OneShot_us(uint32_t delay_us);
void Timer3A_Init_1s(void);
void UART0_Init(void);
void UART0_TxChar(char c);
void UART0_TxString(char *str);

/* ---------- ISRs ---------- */
void TIMER1A_Handler(void);
void TIMER2A_Handler(void);
void TIMER3A_Handler(void);
void GPIOB_Handler(void);

/* ======================================================= */
int main(void) {
    PLL_Init();
    PE1_Digital_Output_Init();
    PE3_ADC_Init();
    PB1_Optocoupler_Init();
    UART0_Init();

    Timer2A_Init_OneShot_us(pulse_width_us);
    Timer1A_Init_Periodic_ms(20U);  // 50 Hz servo control
    Timer3A_Init_1s();              // 1-second RPM sampling

    while (1) {
        // Interrupt-driven operation
    }
}

/* ======================================================= */
/* ---------- UART0 ---------- */
void UART0_Init(void) {
    SYSCTL->RCGCUART |= 0x01;
    SYSCTL->RCGCGPIO |= 0x01;
    for (j = 0; j < 3; j++);

    GPIOA->AFSEL |= 0x03;
    GPIOA->DEN   |= 0x03;
    GPIOA->PCTL  = (GPIOA->PCTL & 0xFFFFFF00) | 0x00000011;
    GPIOA->AMSEL &= ~0x03;

    UART0->CTL &= ~0x01;
    UART0->IBRD = 104;  // 9600 baud @ 16MHz
    UART0->FBRD = 11;
    UART0->LCRH = 0x70;
    UART0->CC = 0x0;
    UART0->CTL = 0x301;
}

void UART0_TxChar(char c) {
    while (UART0->FR & 0x20);
    UART0->DR = c;
}

void UART0_TxString(char *str) {
    while (*str) UART0_TxChar(*str++);
}

/* ======================================================= */
/* ---------- GPIO OUTPUT (Motor) ---------- */
void PE1_Digital_Output_Init(void) {
    SYSCTL->RCGCGPIO |= GPIO_PORTE_CLOCK_EN;
    for (j = 0; j < 3; j++);

    GPIOE->AFSEL &= ~GPIO_PORTE_PIN1;
    GPIOE->DEN   |= GPIO_PORTE_PIN1;
    GPIOE->DIR   |= GPIO_PORTE_PIN1;
    GPIOE->DATA  &= ~GPIO_PORTE_PIN1;
}

/* ======================================================= */
/* ---------- ADC0 on PE3 ---------- */
void PE3_ADC_Init(void) {
    SYSCTL->RCGCADC |= 0x01;
    SYSCTL->RCGCGPIO |= 0x10;
    for (j = 0; j < 3; j++);

    GPIOE->AFSEL |= 0x08;
    GPIOE->DEN &= ~0x08;
    GPIOE->AMSEL |= 0x08;

    ADC0->ACTSS &= ~0x08;
    ADC0->EMUX &= ~0xF000;
    ADC0->SSMUX3 = 0;           // AIN0 (PE3)
    ADC0->SSCTL3 = 0x06;        // IE0 + END0
    ADC0->ACTSS |= 0x08;
}

uint16_t ADC0_Read(void) {
    ADC0->PSSI = 0x08;
    while ((ADC0->RIS & 0x08) == 0);
    uint16_t result = ADC0->SSFIFO3 & 0xFFF;
    ADC0->ISC = 0x08;
    return result;
}

/* ======================================================= */
/* ---------- Optocoupler Input (PB1) ---------- */
void PB1_Optocoupler_Init(void) {
    SYSCTL->RCGCGPIO |= GPIO_PORTB_CLOCK_EN;
    while ((SYSCTL->PRGPIO & GPIO_PORTB_CLOCK_EN) == 0);

    GPIOB->DIR &= ~GPIO_PORTB_PIN1;
    GPIOB->DEN |= GPIO_PORTB_PIN1;
    GPIOB->AFSEL &= ~GPIO_PORTB_PIN1;

    GPIOB->IS  &= ~GPIO_PORTB_PIN1;   // Edge sensitive
    GPIOB->IBE &= ~GPIO_PORTB_PIN1;   // Single edge
    GPIOB->IEV |= GPIO_PORTB_PIN1;    // Rising edge
    GPIOB->ICR |= GPIO_PORTB_PIN1;    // Clear flag
    GPIOB->IM  |= GPIO_PORTB_PIN1;    // Enable interrupt

    NVIC_SetPriority(GPIOB_IRQn, 3);
    NVIC_EnableIRQ(GPIOB_IRQn);
}

/* ======================================================= */
/* ---------- Timers ---------- */

// Timer2A: One-shot pulse width
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

// Timer1A: Periodic 20 ms
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

// Timer3A: 1-second periodic for RPM
void Timer3A_Init_1s(void) {
    SYSCTL->RCGCTIMER |= TIMER3_CLOCK_EN;
    for (j = 0; j < 3; j++);

    TIMER3->CTL = 0x00;
    TIMER3->CFG = 0x00;            // 32-bit
    TIMER3->TAMR = 0x02;           // Periodic
    TIMER3->TAILR = 16000000 - 1;  // 1 second (16MHz)
    TIMER3->ICR = 0x01;
    TIMER3->IMR |= 0x01;

    NVIC_SetPriority(TIMER3A_IRQn, 4);
    NVIC_EnableIRQ(TIMER3A_IRQn);

    TIMER3->CTL |= 0x01;
}

/* ======================================================= */
/* ---------- Interrupts ---------- */

// Servo PWM update
void TIMER1A_Handler(void) {
    uint16_t adc_value = ADC0_Read();
    pulse_width_us = 1000 + (adc_value * 200) / 4095;

    TIMER2->TAILR = pulse_width_us - 1;
    GPIOE->DATA |= GPIO_PORTE_PIN1;
    TIMER2->CTL |= 0x01;

    TIMER1->ICR = 0x01;
}

// End of one-shot PWM
void TIMER2A_Handler(void) {
    TIMER2->ICR = 0x01;
    GPIOE->DATA &= ~GPIO_PORTE_PIN1;
    TIMER2->CTL &= ~0x01;
}

// 1-second RPM counter
void TIMER3A_Handler(void) {
    char buffer[40];
    rpm = pulse_count * 60;      // 1 pulse per revolution
    pulse_count = 0;

    sprintf(buffer, "RPM:%lu\n", rpm);
    UART0_TxString(buffer);

    TIMER3->ICR = 0x01;
}

// Optocoupler edge detect
void GPIOB_Handler(void) {
    if (GPIOB->MIS & GPIO_PORTB_PIN1) {
        pulse_count++;
        GPIOB->ICR |= GPIO_PORTB_PIN1;
    }
}

/* ======================================================= */
/* ---------- PLL ---------- */
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