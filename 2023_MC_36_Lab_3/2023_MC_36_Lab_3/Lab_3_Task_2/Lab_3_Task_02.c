#include "TM4C123.h"
#include <stdint.h>

volatile uint32_t press_start = 0;
volatile uint32_t press_time_ms = 0;
volatile uint8_t button_pressed = 0;

uint8_t seg_code[10] = {0x3F,0x06,0x5B,0x4F,0x66,
                        0x6D,0x7D,0x07,0x7F,0x6F};

void PortB_Init(void);
void PortE_Init(void);
void PortF_Init(void);
void Timer0A_Init(void);
void GPIOF_Handler(void);
void displayNumber(uint32_t num);
void DelayMs(uint32_t n);

int main(void)
{
    PortB_Init();
    PortE_Init();
    PortF_Init();
    Timer0A_Init();

    while(1)
    {
        displayNumber(press_time_ms);
    }
}

/* ---------- Initialize PortB for segments ---------- */
void PortB_Init(void)
{
    SYSCTL->RCGCGPIO |= (1<<1);
    while((SYSCTL->PRGPIO & (1<<1))==0);
    GPIOB->DIR |= 0x7F;   // PB0–PB6 output
    GPIOB->DEN |= 0x7F;
}

/* ---------- Initialize PortE for digit control ---------- */
void PortE_Init(void)
{
    SYSCTL->RCGCGPIO |= (1<<4);
    while((SYSCTL->PRGPIO & (1<<4))==0);
    GPIOE->DIR |= 0x0F;   // PE0–PE3 output
    GPIOE->DEN |= 0x0F;
}

/* ---------- Initialize PortF for pushbutton ---------- */
void PortF_Init(void)
{
    SYSCTL->RCGCGPIO |= (1<<5);
    while((SYSCTL->PRGPIO & (1<<5))==0);
    GPIOF->DIR &= ~(1<<0);    // PF0 input
    GPIOF->DEN |= (1<<0);

    // Interrupt setup for both edges
    GPIOF->IS  &= ~(1<<0);    // Edge-sensitive
    GPIOF->IBE |=  (1<<0);    // Both edges
    GPIOF->ICR  |=  (1<<0);   // Clear prior
    GPIOF->IM  |=  (1<<0);    // Unmask

    NVIC->ISER[0] |= (1<<30); // Enable GPIOF interrupt
}

/* ---------- Initialize Timer0A in 1ms periodic mode ---------- */
void Timer0A_Init(void)
{
    SYSCTL->RCGCTIMER |= (1<<0);
    TIMER0->CTL = 0;
    TIMER0->CFG = 0;
    TIMER0->TAMR = 0x02;        // periodic mode
    TIMER0->TAILR = 16000 - 1;  // 1ms at 16 MHz
    TIMER0->ICR = 0x1;
    TIMER0->CTL |= 0x01;
}

/* ---------- GPIOF Interrupt Handler ---------- */
void GPIOF_Handler(void)
{
    if(GPIOF->MIS & (1<<0))
    {
        GPIOF->ICR |= (1<<0);

        if(GPIOF->DATA & (1<<0))   // Button pressed
        {
            press_start = TIMER0->TAV; // start count
            button_pressed = 1;
        }
        else                        // Button released
        {
            if(button_pressed)
            {
                uint32_t press_end = TIMER0->TAV;
                if(press_end >= press_start)
                    press_time_ms = (press_end - press_start) / 16000; // in ms
                else
                    press_time_ms = (0xFFFFFFFF - press_start + press_end) / 16000;
                button_pressed = 0;
            }
        }
    }
}

/* ---------- Display Number on 4-digit 7-segment ---------- */
void displayNumber(uint32_t num)
{
    uint8_t digits[4];
    digits[0] = num % 10;
    digits[1] = (num/10)%10;
    digits[2] = (num/100)%10;
    digits[3] = (num/1000)%10;

    for(int i=0;i<4;i++)
    {
        GPIOE->DATA = (1<<i);       // Enable one digit
        GPIOB->DATA = seg_code[digits[i]];
        DelayMs(5);
        GPIOE->DATA = 0x00;         // Disable all
    }
}

/* ---------- Simple Delay ---------- */
void DelayMs(uint32_t n)
{
    volatile uint32_t i, j;
    for(i=0;i<n;i++)
        for(j=0;j<16000;j++);
}
