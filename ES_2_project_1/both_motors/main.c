#include "TM4C123.h"
#include <stdint.h>
#include <stdio.h>

#define SYSCLK 16000000UL

// === Pin Definitions ===
#define ESC1_PIN (1U << 6)  // PB6 -> Motor 1
#define ESC2_PIN (1U << 5)  // PB5 -> Motor 2

// === Globals ===
volatile uint32_t pulse1_us = 1000;
volatile uint32_t pulse2_us = 1000;
volatile uint32_t rpm1 = 0, rpm2 = 0;
volatile uint32_t count1 = 0, count2 = 0;

// === Function Prototypes ===
void PLL_Init(void);
void GPIO_Init(void);
void ADC0_Init(void);
uint16_t ADC0_Read(uint8_t channel);
void Timer0A_50Hz_Init(void);
void Timer1A_Pulse1_Init(void);
void Timer2A_Pulse2_Init(void);
void UART0_Init(void);
void UART0_SendString(char *str);
void set_pulse1_us(uint32_t us);
void set_pulse2_us(uint32_t us);
void delayMs(uint32_t n);

// === MAIN ===
int main(void) {
    PLL_Init();
    GPIO_Init();
    ADC0_Init();
    UART0_Init();
    Timer1A_Pulse1_Init();
    Timer2A_Pulse2_Init();
    Timer0A_50Hz_Init();
    __enable_irq();

    set_pulse1_us(1000);
    set_pulse2_us(1000);
    delayMs(2000); // ESC arm delay

    float val1_f = 0, val2_f = 0;
    uint32_t prev1 = 0, prev2 = 0;

    while (1) {
        uint16_t val1 = ADC0_Read(0); // PE3
        uint16_t val2 = ADC0_Read(3); // PE0

        val1_f = 0.9f * val1_f + 0.1f * val1;
        val2_f = 0.9f * val2_f + 0.1f * val2;

        pulse1_us = 1000 + ((uint32_t)(val1_f * 500) / 4095);
        pulse2_us = 1000 + ((uint32_t)(val2_f * 500) / 4095);
        if (pulse1_us < 1050) pulse1_us = 1000;
        if (pulse2_us < 1050) pulse2_us = 1000;

        // RPM calculation every 500ms
        delayMs(500);
        rpm1 = (count1 - prev1) * 120; // (pulses/0.5s)*60
        rpm2 = (count2 - prev2) * 120;
        prev1 = count1;
        prev2 = count2;

        char buffer[100];
        sprintf(buffer, "RPM1=%lu, RPM2=%lu\r\n", rpm1, rpm2);
        UART0_SendString(buffer);
    }
}

// === GPIO & Laser Sensor ===
void GPIO_Init(void) {
    SYSCTL->RCGCGPIO |= (1U << 0) | (1U << 1) | (1U << 4); // Ports A, B, E
    for (volatile int i = 0; i < 10; i++);

    // PB5, PB6 -> Outputs for ESC
    GPIOB->DIR |= ESC1_PIN | ESC2_PIN;
    GPIOB->DEN |= ESC1_PIN | ESC2_PIN;
    GPIOB->DATA &= ~(ESC1_PIN | ESC2_PIN);

    // PE3, PE0 -> ADC
    GPIOE->AFSEL |= (1U << 3) | (1U << 0);
    GPIOE->DEN &= ~((1U << 3) | (1U << 0));
    GPIOE->AMSEL |= (1U << 3) | (1U << 0);

    // PA6, PA7 -> Laser Sensors (Inputs with Interrupt)
    GPIOA->DIR &= ~((1U << 6) | (1U << 7));
    GPIOA->DEN |= (1U << 6) | (1U << 7);
    GPIOA->IS &= ~((1U << 6) | (1U << 7));   // Edge-sensitive
    GPIOA->IBE &= ~((1U << 6) | (1U << 7));  // Single edge
    GPIOA->IEV |= (1U << 6) | (1U << 7);     // Rising edge
    GPIOA->ICR |= (1U << 6) | (1U << 7);
    GPIOA->IM |= (1U << 6) | (1U << 7);
    NVIC->ISER[0] |= (1U << 0);              // GPIOA interrupt
}

// === Laser Sensor ISR ===
void GPIOA_Handler(void) {
    if (GPIOA->RIS & (1U << 6)) {
        count1++;
        GPIOA->ICR |= (1U << 6);
    }
    if (GPIOA->RIS & (1U << 7)) {
        count2++;
        GPIOA->ICR |= (1U << 7);
    }
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
    UART0->IBRD = 104; // 9600 baud
    UART0->FBRD = 11;
    UART0->LCRH = 0x70;
    UART0->CTL = 0x301;
    GPIOA->AFSEL |= (1U << 0) | (1U << 1);
    GPIOA->PCTL = (GPIOA->PCTL & ~0xFF) | 0x11;
    GPIOA->DEN |= (1U << 0) | (1U << 1);
}

void UART0_SendChar(char c) {
    while ((UART0->FR & 0x20) != 0);
    UART0->DR = c;
}
void UART0_SendString(char *str) {
    while (*str) UART0_SendChar(*str++);
}

// === PWM Timers ===
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

void Timer1A_Pulse1_Init(void) {
    SYSCTL->RCGCTIMER |= (1U << 1);
    TIMER1->CTL = 0;
    TIMER1->CFG = 0x04;
    TIMER1->TAMR = 0x01;
    TIMER1->ICR = 0x1;
    TIMER1->IMR = 0x1;
    NVIC->ISER[0] |= (1U << 21);
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

// === PWM ISRs ===
void TIMER0A_Handler(void) {
    TIMER0->ICR = 0x1;
    GPIOB->DATA |= ESC1_PIN;
    TIMER1->TAILR = (SYSCLK / 1000000) * pulse1_us - 1;
    TIMER1->CTL |= 0x1;

    GPIOB->DATA |= ESC2_PIN;
    TIMER2->TAILR = (SYSCLK / 1000000) * pulse2_us - 1;
    TIMER2->CTL |= 0x1;
}

void TIMER1A_Handler(void) {
    TIMER1->ICR = 0x1;
    GPIOB->DATA &= ~ESC1_PIN;
}

void TIMER2A_Handler(void) {
    TIMER2->ICR = 0x1;
    GPIOB->DATA &= ~ESC2_PIN;
}

void set_pulse1_us(uint32_t us) {
    if (us < 1000) us = 1000;
    if (us > 1500) us = 1500;
    pulse1_us = us;
}

void set_pulse2_us(uint32_t us) {
    if (us < 1000) us = 1000;
    if (us > 1500) us = 1500;
    pulse2_us = us;
}

// === Delay & PLL ===
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