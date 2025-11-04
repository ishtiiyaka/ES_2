#include "TM4C123.h"
#include <stdint.h>
#include <stdio.h>

#define SYSCLK 16000000UL
#define PULSES_PER_REV 1

// === Pin Definitions ===
#define ESC1_PIN (1U << 6)  // PB6 -> Motor 1 (fixed RPM)
#define ESC2_PIN (1U << 7)  // PB7 -> Motor 2 (potentiometer)

// === Globals ===
volatile uint32_t pulse1_us = 1000;  // Motor 1 pulse width
volatile uint32_t pulse2_us = 1000;  // Motor 2 pulse width
volatile uint32_t rpm1 = 0, rpm2 = 0;
volatile uint32_t period1_ticks = 0, period2_ticks = 0;
volatile uint8_t newPeriod1 = 0, newPeriod2 = 0;

// === Function Prototypes ===
void PLL_Init(void);
void GPIO_Init(void);
void ADC0_Init(void);
uint16_t ADC0_Read(uint8_t channel);
void Timer0A_50Hz_Init(void);
void Timer1_Capture_Init(void);
void Timer3A_Pulse1_Init(void);
void Timer2A_Pulse2_Init(void);
void UART0_Init(void);
void UART0_SendChar(char c);
void UART0_SendString(char *str);
void delayMs(uint32_t n);
float computeRPM(uint32_t ticks);
uint32_t RPMtoPulse(uint32_t target_rpm);

// === MAIN ===
int main(void) {
    PLL_Init();
    GPIO_Init();
    ADC0_Init();
    UART0_Init();
    UART0_SendString("UART0 Ready\r\n");

    Timer1_Capture_Init();
    Timer3A_Pulse1_Init();
    Timer2A_Pulse2_Init();
    Timer0A_50Hz_Init();
    __enable_irq();

    delayMs(2000);  // ESC arm delay

    char buffer[100];
    float rpm_f1 = 0, rpm_f2 = 0;
    float val2_f = 0;

    while (1) {
        // === Fixed RPM for Motor 1 ===
        pulse1_us = 1300;  // 2000 RPM fixed

        // === Read potentiometer for Motor 2 ===
        uint16_t val2 = ADC0_Read(3); // PE0 -> AIN3
        val2_f = 0.9f * val2_f + 0.1f * val2;
        pulse2_us = 1000 + ((uint32_t)(val2_f * 1000) / 4095); // 1-2 ms
        if (pulse2_us < 1000) pulse2_us = 1000;
        if (pulse2_us > 2000) pulse2_us = 2000;

        // === Compute RPMs ===
        if (newPeriod1) {
            rpm_f1 = computeRPM(period1_ticks);
            rpm1 = (uint32_t)rpm_f1;
            newPeriod1 = 0;
        }
        if (newPeriod2) {
            rpm_f2 = computeRPM(period2_ticks);
            rpm2 = (uint32_t)rpm_f2;
            newPeriod2 = 0;
        }

        // === Send UART line once per second ===
        sprintf(buffer, "RPM1=%lu, RPM2=%lu\r\n", rpm1, rpm2);
        UART0_SendString(buffer);

        delayMs(1000); // 1 second delay
    }
}

// === Convert target RPM to pulse width (us) ===
uint32_t RPMtoPulse(uint32_t target_rpm) {
    // Assuming 1000-2000us corresponds linearly to 0-5000 RPM max
    // Adjust scaling as needed
    if (target_rpm > 5000) target_rpm = 5000;
    return 1000 + (target_rpm * 1000 / 5000);
}

// === GPIO Setup ===
void GPIO_Init(void) {
    SYSCTL->RCGCGPIO |= (1U << 0) | (1U << 1) | (1U << 4); // Ports A,B,E
    for (volatile int i = 0; i < 10; i++);

    GPIOB->DIR |= ESC1_PIN | ESC2_PIN;
    GPIOB->DEN |= ESC1_PIN | ESC2_PIN;
    GPIOB->DATA &= ~(ESC1_PIN | ESC2_PIN);

    // PB4, PB5 capture inputs
    GPIOB->AFSEL |= (1U << 4) | (1U << 5);
    GPIOB->PCTL &= ~((0xF << (4*4)) | (0xF << (5*4)));
    GPIOB->PCTL |= (0x7 << (4*4)) | (0x7 << (5*4));
    GPIOB->DEN |= (1U << 4) | (1U << 5);
    GPIOB->DIR &= ~((1U << 4) | (1U << 5));
    GPIOB->PUR |= (1U << 4) | (1U << 5);

    // PE3 and PE0 analog
    GPIOE->AFSEL |= (1U << 3) | (1U << 0);
    GPIOE->DEN &= ~((1U << 3) | (1U << 0));
    GPIOE->AMSEL |= (1U << 3) | (1U << 0);
}

// === ADC ===
void ADC0_Init(void) {
    SYSCTL->RCGCADC |= 1;
    for (volatile int i = 0; i < 3; i++);
    ADC0->ACTSS &= ~8;
    ADC0->EMUX &= ~0xF000;
    ADC0->SSMUX3 = 0;
    ADC0->SSCTL3 = 0x06;
    ADC0->ACTSS |= 8;
}

uint16_t ADC0_Read(uint8_t channel) {
    ADC0->ACTSS &= ~8;
    ADC0->SSMUX3 = channel;
    ADC0->ACTSS |= 8;
    ADC0->PSSI |= 8;
    while ((ADC0->RIS & 8) == 0);
    uint16_t result = ADC0->SSFIFO3 & 0xFFF;
    ADC0->ISC = 8;
    return result;
}

// === UART ===
void UART0_Init(void) {
    SYSCTL->RCGCUART |= 1;
    SYSCTL->RCGCGPIO |= 1;
    UART0->CTL &= ~0x01;
    UART0->IBRD = 104;
    UART0->FBRD = 11;
    UART0->LCRH = (0x3 << 5);
    UART0->CC = 0x0;
    UART0->CTL = 0x301;
    GPIOA->AFSEL |= (1U << 0) | (1U << 1);
    GPIOA->PCTL = (GPIOA->PCTL & ~0xFF) | 0x11;
    GPIOA->DEN |= (1U << 0) | (1U << 1);
}

void UART0_SendChar(char c) {
    while (UART0->FR & 0x20);
    UART0->DR = c;
}

void UART0_SendString(char *str) {
    while (*str) UART0_SendChar(*str++);
}

// === ESC PWM Timers ===
void Timer0A_50Hz_Init(void) {
    SYSCTL->RCGCTIMER |= (1U << 0);
    TIMER0->CTL = 0;
    TIMER0->CFG = 0;
    TIMER0->TAMR = 0x02;
    TIMER0->TAILR = SYSCLK / 50 - 1;
    TIMER0->ICR = 0x1;
    TIMER0->IMR = 0x1;
    NVIC->ISER[0] |= (1U << 19);
    TIMER0->CTL |= 0x1;
}

void TIMER0A_Handler(void) {
    TIMER0->ICR = 0x1;
    GPIOB->DATA |= ESC1_PIN;
    TIMER3->TAILR = (SYSCLK / 1000000) * pulse1_us - 1;
    TIMER3->CTL |= 0x1;
    GPIOB->DATA |= ESC2_PIN;
    TIMER2->TAILR = (SYSCLK / 1000000) * pulse2_us - 1;
    TIMER2->CTL |= 0x1;
}

void Timer3A_Pulse1_Init(void) {
    SYSCTL->RCGCTIMER |= (1U << 3);
    TIMER3->CTL = 0;
    TIMER3->CFG = 0x04;
    TIMER3->TAMR = 0x01;
    TIMER3->ICR = 0x1;
    TIMER3->IMR = 0x1;
    NVIC->ISER[1] = (1U << (35-32));
}

void TIMER3A_Handler(void) {
    TIMER3->ICR = 0x1;
    GPIOB->DATA &= ~ESC1_PIN;
}

void Timer2A_Pulse2_Init(void) {
    SYSCTL->RCGCTIMER |= (1U << 2);
    TIMER2->CTL = 0;
    TIMER2->CFG = 0x04;
    TIMER2->TAMR = 0x01;
    TIMER2->ICR = 0x1;
    TIMER2->IMR = 0x1;
    NVIC->ISER[0] |= (1U << 23);
}

void TIMER2A_Handler(void) {
    TIMER2->ICR = 0x1;
    GPIOB->DATA &= ~ESC2_PIN;
}

// === Capture Timer1 ===
void Timer1_Capture_Init(void) {
    SYSCTL->RCGCTIMER |= (1U << 1);
    while ((SYSCTL->PRTIMER & (1U << 1)) == 0);

    TIMER1->CTL &= ~(1U << 0);
    TIMER1->CFG = 0x04;
    TIMER1->TAMR = 0x17;
    TIMER1->CTL &= ~(1U << 2);
    TIMER1->IMR |= (1U << 2);

    TIMER1->CTL &= ~(1U << 8);
    TIMER1->TBMR = 0x17;
    TIMER1->CTL &= ~(1U << 10);
    TIMER1->IMR |= (1U << 10);

    TIMER1->CTL |= (1U << 0) | (1U << 8);
    NVIC->ISER[0] |= (1U << 21);
    NVIC->ISER[0] |= (1U << 22);
}

void TIMER1A_Handler(void) {
    static uint32_t lastCap1 = 0;
    uint32_t now = TIMER1->TAR;
    period1_ticks = (now >= lastCap1) ? (now - lastCap1) : (0xFFFFFFFF - lastCap1 + now);
    lastCap1 = now;
    newPeriod1 = 1;
    TIMER1->ICR = (1U << 2);
}

void TIMER1B_Handler(void) {
    static uint32_t lastCap2 = 0;
    uint32_t now = TIMER1->TBR;
    period2_ticks = (now >= lastCap2) ? (now - lastCap2) : (0xFFFFFFFF - lastCap2 + now);
    lastCap2 = now;
    newPeriod2 = 1;
    TIMER1->ICR = (1U << 10);
}

// === Helper Functions ===
float computeRPM(uint32_t ticks) {
    if (ticks == 0) return 0;
    float period_s = (float)ticks / SYSCLK;
    float freq = 1.0f / period_s;
    return freq * 60.0f / PULSES_PER_REV;
}

void delayMs(uint32_t n) {
    for (uint32_t i = 0; i < n * 3180; i++);
}

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
