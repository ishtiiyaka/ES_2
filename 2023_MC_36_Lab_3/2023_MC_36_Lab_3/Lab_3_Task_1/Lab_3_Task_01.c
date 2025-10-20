#include "TM4C123.h"
#include <stdio.h>

volatile unsigned int pulse_count = 0;
volatile uint32_t throttle_pulse = 1500;  
volatile uint32_t min_throttle = 1000;
volatile uint32_t max_throttle = 2000;
volatile uint32_t adcValue = 0;
volatile uint8_t esc_calibrated = 0;

void PLL_Init(void);
void GPIOA_Init(void);
void PB3_Init(void);
void UART0_Init(void);
void UART0_TxChar(char c);
void UART0_TxString(char *str);
void Timer0A_Init(void);
void Timer1A_Init(void);
void ADC0_Init(void);
void DelayMs(uint32_t n);
void ADC0SS3_Handler(void);
void TIMER0A_Handler(void);
void TIMER1A_Handler(void);
void GPIOA_Handler(void);

int main(void)
{
    PLL_Init();
    GPIOA_Init();
    UART0_Init();
    PB3_Init();
    ADC0_Init();
    Timer0A_Init();
    Timer1A_Init();

    UART0_TxString("ESC Motor Control (Interrupt-Based)\r\n");

    // --- ESC ARMING SEQUENCE (done once at startup) ---
    throttle_pulse = max_throttle;  // Max throttle pulse
    DelayMs(300);
    throttle_pulse = min_throttle;  // Min throttle pulse
    DelayMs(300);
    esc_calibrated = 1;

    // Start ADC sampling (triggered in software periodically)
    ADC0->PSSI |= (1 << 3);

    while (1)
    {
        // Empty loop - everything runs in interrupt context
    }
}

// ---------- PB3: PWM output ----------
void PB3_Init(void)
{
    SYSCTL->RCGCGPIO |= (1<<1);     
    while((SYSCTL->PRGPIO & (1<<1))==0);
    GPIOB->DIR |= (1<<3);           
    GPIOB->DEN |= (1<<3);           
    GPIOB->AFSEL &= ~(1<<3);        
    GPIOB->DATA &= ~(1<<3);         
}

// ---------- ADC0 (PE3/AIN0) ----------
void ADC0_Init(void)
{
    SYSCTL->RCGCADC |= (1<<0);
    SYSCTL->RCGCGPIO |= (1<<4);
    while((SYSCTL->PRGPIO & (1<<4))==0);
    GPIOE->AFSEL |= (1<<3);
    GPIOE->DEN &= ~(1<<3);
    GPIOE->AMSEL |= (1<<3);
    
    ADC0->ACTSS &= ~(1<<3);
    ADC0->EMUX &= ~0xF000;         // Software trigger
    ADC0->SSMUX3 = 0;
    ADC0->SSCTL3 = 0x06;
    ADC0->IM |= (1<<3);            // Enable SS3 interrupt
    NVIC_EnableIRQ(ADC0SS3_IRQn);
    ADC0->ACTSS |= (1<<3);
}

void ADC0SS3_Handler(void)
{
    ADC0->ISC = (1<<3);
    adcValue = ADC0->SSFIFO3 & 0xFFF;

    // Map ADC value (0–4095) ? 1000–2000 µs pulse width
    throttle_pulse = min_throttle + ((adcValue * (max_throttle - min_throttle)) / 4095);

    // Request next conversion
    ADC0->PSSI |= (1 << 3);
}

// ---------- TIMER0A: 20 ms period ----------
void Timer0A_Init(void)
{
    SYSCTL->RCGCTIMER |= (1<<0);
    TIMER0->CTL = 0;
    TIMER0->CFG = 0x00;
    TIMER0->TAMR = 0x02;                 // Periodic mode
    TIMER0->TAILR = 16000*20 - 1;        // 20 ms
    TIMER0->IMR |= 0x01;
    NVIC_EnableIRQ(TIMER0A_IRQn);
    TIMER0->CTL |= 0x01;
}

// ---------- TIMER1A: one-shot pulse width ----------
void Timer1A_Init(void)
{
    SYSCTL->RCGCTIMER |= (1<<1);
    TIMER1->CTL = 0;
    TIMER1->CFG = 0x00;
    TIMER1->TAMR = 0x01;    // One-shot mode
    TIMER1->IMR |= 0x01;
    NVIC_EnableIRQ(TIMER1A_IRQn);
}

void TIMER0A_Handler(void)
{
    TIMER0->ICR = 0x01;
    GPIOB->DATA |= (1<<3);  // PWM HIGH
    TIMER1->TAILR = 16 * throttle_pulse - 1;  // µs ? clock ticks
    TIMER1->CTL |= 0x01;    // Start one-shot timer

    // Trigger ADC read every PWM cycle
    ADC0->PSSI |= (1 << 3);
}

void TIMER1A_Handler(void)
{
    TIMER1->ICR = 0x01;
    GPIOB->DATA &= ~(1<<3);  // PWM LOW
}

// ---------- PLL: 16 MHz ----------
void PLL_Init(void)
{
    SYSCTL->RCC2 |= 0x80000000;
    SYSCTL->RCC2 |= 0x00000800;
    SYSCTL->RCC = (SYSCTL->RCC & ~0x000007C0) + 0x00000540;
    SYSCTL->RCC2 &= ~0x00000070;
    SYSCTL->RCC2 &= ~0x00002000;
    SYSCTL->RCC2 |= 0x40000000;
    SYSCTL->RCC2 = (SYSCTL->RCC2 & ~0x1FC00000) + (24 << 22);
    while((SYSCTL->RIS & 0x00000040)==0);
    SYSCTL->RCC2 &= ~0x00000800;
}

// ---------- UART ----------
void UART0_Init(void)
{
    SYSCTL->RCGCUART |= (1 << 0);
    SYSCTL->RCGCGPIO |= (1 << 0);
    while ((SYSCTL->PRUART & 0x01) == 0);

    GPIOA->AFSEL |= 0x03;
    GPIOA->PCTL &= ~0xFF;
    GPIOA->PCTL |= 0x11;
    GPIOA->DEN |= 0x03;
    GPIOA->AMSEL &= ~0x03;

    UART0->CTL &= ~0x01;
    UART0->IBRD = 104;
    UART0->FBRD = 11;
    UART0->LCRH = 0x60;
    UART0->CC = 0x5;
    UART0->CTL = 0x301;
}

void UART0_TxChar(char c)
{
    while (UART0->FR & 0x20);
    UART0->DR = c;
}

void UART0_TxString(char *str)
{
    while (*str)
        UART0_TxChar(*str++);
}

// ---------- PORT A Interrupt (Optocoupler) ----------
void GPIOA_Init(void)
{
    SYSCTL->RCGCGPIO |= (1 << 0);
    while ((SYSCTL->PRGPIO & (1 << 0)) == 0);

    GPIOA->DIR &= ~(1 << 3);
    GPIOA->DEN |= (1 << 3);
    GPIOA->PUR |= (1 << 3);

    GPIOA->IS &= ~(1 << 3);
    GPIOA->IBE &= ~(1 << 3);
    GPIOA->IEV &= ~(1 << 3);
    GPIOA->ICR |= (1 << 3);
    GPIOA->IM |= (1 << 3);

    NVIC->ISER[0] |= (1 << 0);
}

void GPIOA_Handler(void)
{
    if (GPIOA->MIS & (1 << 3))
    {
        GPIOA->ICR |= (1 << 3);
        pulse_count++;
    }
}

// ---------- Delay ----------
void DelayMs(uint32_t n)
{
    volatile uint32_t i, j;
    for(i=0;i<n;i++)
        for(j=0;j<16000;j++);
}