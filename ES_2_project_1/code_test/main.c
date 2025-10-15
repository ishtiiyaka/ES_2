#include "TM4C123.h"
#include <stdint.h>

/* ---------- PE1 (Motor Pulse Output) ---------- */
#define GPIO_PORTE_CLOCK_EN  0x10
#define GPIO_PORTE_PIN1      0x02
#define GPIO_PORTE_PIN1_OP   0x02
#define GPIO_PORTE_PIN1_EN   0x02

/* ---------- PF2 (Blue LED Output) ---------- */
#define GPIO_PORTF_CLOCK_EN  0x20
#define GPIO_PORTF_PIN2_OP   0x04
#define GPIO_PORTF_PIN2_EN   0x04
#define GPIO_PORTF_PIN2      0x04

/* ---------- PB1 (Sensor Input) ---------- */
#define GPIO_PORTB_CLOCK_EN  0x02
#define GPIO_PORTB_PIN1_IN   0x02
#define GPIO_PORTB_PIN1_EN   0x02
#define GPIO_PORTB_PIN1      0x02

/* ---------- Timer Numbers ---------- */
#define TIMER1_CLOCK_EN      (1<<1)
#define TIMER2_CLOCK_EN      (1<<2)
#define TIMER3_CLOCK_EN      (1<<3)

/* ---------- Function Prototypes ---------- */
void PLL_Init(void);
void PE1_Digital_Output_Init(void);
void PF2_Digital_Output_Init(void);
void PB1_Digital_Input_Init(void);
void Timer1A_Init_Periodic_ms(uint32_t period_ms);
void Timer2A_Init_OneShot_us(uint32_t delay_us);
void Timer3A_OneShot_Init(void);
void Timer3A_Delay(uint32_t cycles);

/* ---------- ISR Prototypes ---------- */
void TIMER1A_Handler(void);
void TIMER2A_Handler(void);

/* ---------- Globals ---------- */
unsigned volatile long j;

/* ======================================================= */
int main(void){
    PLL_Init();                        // Set system clock to 16 MHz

    /* --- Initialization --- */
    PE1_Digital_Output_Init();         // Motor PWM output
    PF2_Digital_Output_Init();         // Blue LED output
    PB1_Digital_Input_Init();          // Sensor input

    Timer2A_Init_OneShot_us(1200U);    // 1500 ?s pulse width
    Timer1A_Init_Periodic_ms(20U);     // 20 ms period (50 Hz)
    Timer3A_OneShot_Init();            // For 1-second LED delay

    while(1){
        /* --- Sensor Polling (PB1) --- */
        if(GPIOB->DATA & GPIO_PORTB_PIN1){
            GPIOF->DATA |= GPIO_PORTF_PIN2;     // Turn ON Blue LED
            Timer3A_Delay(16000000);            // 1 second delay (16 MHz)
            GPIOF->DATA &= ~GPIO_PORTF_PIN2;    // Turn OFF Blue LED
        }
    }
}

/* ======================================================= */
/* ---------- GPIO Configurations ---------- */

void PE1_Digital_Output_Init(void){
    SYSCTL->RCGCGPIO |= GPIO_PORTE_CLOCK_EN;
    for(j=0;j<3;j++);

    GPIOE->AFSEL &= ~GPIO_PORTE_PIN1_OP;
    GPIOE->DEN   |= GPIO_PORTE_PIN1_EN;
    GPIOE->DIR   |= GPIO_PORTE_PIN1_OP;
    GPIOE->DATA  &= ~GPIO_PORTE_PIN1;
}

void PF2_Digital_Output_Init(void){
    SYSCTL->RCGCGPIO |= GPIO_PORTF_CLOCK_EN;
    while((SYSCTL->PRGPIO & GPIO_PORTF_CLOCK_EN)==0);
    GPIOF->AFSEL &= ~GPIO_PORTF_PIN2;
    GPIOF->DEN   |=  GPIO_PORTF_PIN2_EN;
    GPIOF->DIR   |=  GPIO_PORTF_PIN2_OP;
}

void PB1_Digital_Input_Init(void){
    SYSCTL->RCGCGPIO |= GPIO_PORTB_CLOCK_EN;
    while((SYSCTL->PRGPIO & GPIO_PORTB_CLOCK_EN)==0);
    GPIOB->AFSEL &= ~GPIO_PORTB_PIN1;
    GPIOB->DEN   |=  GPIO_PORTB_PIN1_EN;
    GPIOB->DIR   &= ~GPIO_PORTB_PIN1_IN;
}

/* ======================================================= */
/* ---------- Timer2A One-Shot (Motor Pulse Width) ---------- */
void Timer2A_Init_OneShot_us(uint32_t delay_us){
    SYSCTL->RCGCTIMER |= TIMER2_CLOCK_EN;
    for(j=0;j<3;j++);

    TIMER2->CTL = 0x00;
    TIMER2->CFG = 0x04;                // 16-bit timer
    TIMER2->TAMR = 0x01;               // One-shot
    TIMER2->TAPR = 16 - 1;             // 1 ?s tick (16MHz / 16)
    TIMER2->TAILR = delay_us - 1;
    TIMER2->ICR = 0x01;
    TIMER2->IMR |= 0x01;

    NVIC_SetPriority(TIMER2A_IRQn, 2);
    NVIC_EnableIRQ(TIMER2A_IRQn);
}

/* ---------- Timer1A Periodic (Motor Pulse Repetition) ---------- */
void Timer1A_Init_Periodic_ms(uint32_t period_ms){
    SYSCTL->RCGCTIMER |= TIMER1_CLOCK_EN;
    for(j=0;j<3;j++);

    TIMER1->CTL = 0x00;
    TIMER1->CFG = 0x04;
    TIMER1->TAMR = 0x02;               // Periodic
    TIMER1->TAPR = 250 - 1;            // 1 ms tick
    TIMER1->TAILR = (64 * period_ms) - 1;
    TIMER1->ICR = 0x01;
    TIMER1->IMR |= 0x01;

    NVIC_SetPriority(TIMER1A_IRQn, 1);
    NVIC_EnableIRQ(TIMER1A_IRQn);

    TIMER1->CTL |= 0x01;               // Start Timer1A
}

/* ---------- Timer3A One-Shot (LED Delay) ---------- */
void Timer3A_OneShot_Init(void){
    SYSCTL->RCGCTIMER |= TIMER3_CLOCK_EN;
    while((SYSCTL->PRTIMER & TIMER3_CLOCK_EN)==0);
    TIMER3->CTL &= ~0x01;
    TIMER3->CFG = 0x04;
    TIMER3->TAMR = 0x01;               // One-shot
    TIMER3->CTL &= ~0x01;
}

/* ---------- Timer3A Delay Function ---------- */
void Timer3A_Delay(uint32_t cycles){
    TIMER3->TAILR = cycles - 1;
    TIMER3->ICR = 0x01;
    TIMER3->CTL |= 0x01;
    while((TIMER3->RIS & 0x01)==0);
    TIMER3->ICR = 0x01;
}

/* ======================================================= */
/* ---------- ISRs ---------- */
void TIMER1A_Handler(void){
    GPIOE->DATA |= GPIO_PORTE_PIN1;    // Start pulse
    TIMER2->CTL |= 0x01;               // Start one-shot
    TIMER1->ICR = 0x01;                // Clear flag
}

void TIMER2A_Handler(void){
    TIMER2->ICR = 0x01;                // Clear flag
    GPIOE->DATA &= ~GPIO_PORTE_PIN1;   // End pulse
    TIMER2->CTL &= ~0x01;              // Stop timer
}

/* ======================================================= */
/* ---------- PLL Init ---------- */
void PLL_Init(void){
    SYSCTL->RCC2 |=  0x80000000;       // Use RCC2
    SYSCTL->RCC2 |=  0x00000800;       // BYPASS2
    SYSCTL->RCC  =  (SYSCTL->RCC &~0x000007C0) + 0x00000540;
    SYSCTL->RCC2 &= ~0x00000070;
    SYSCTL->RCC2 &= ~0x00002000;
    SYSCTL->RCC2 |=  0x40000000;
    SYSCTL->RCC2 =  (SYSCTL->RCC2 &~0x1FC00000) + (24<<22);
    while((SYSCTL->RIS & 0x00000040)==0);
    SYSCTL->RCC2 &= ~0x00000800;
}