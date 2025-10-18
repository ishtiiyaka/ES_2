#include "TM4C123.h"
#include <stdint.h>

/* =======================================================
   ---------- PE1 (Motor Pulse Output) ----------
   PE1 outputs PWM-like pulses to control an ESC or motor.
   ======================================================= */
#define GPIO_PORTE_CLOCK_EN   0x10
#define GPIO_PORTE_PIN1       0x02
#define GPIO_PORTE_PIN1_OP    0x02
#define GPIO_PORTE_PIN1_EN    0x02

/* =======================================================
   ---------- Timer Clock Enables ----------
   TIMER1 – Periodic 20 ms interrupt (PWM frame)
   TIMER2 – One-shot for pulse width (1–2 ms)
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

/* ---------- ISR Prototypes ---------- */
void TIMER1A_Handler(void);
void TIMER2A_Handler(void);

/* ---------- Globals ---------- */
volatile uint32_t pulse_width_us = 0;  // Pulse width (microseconds)
unsigned volatile long j;

/* =======================================================
   ---------- MAIN FUNCTION ----------
   ======================================================= */
int main(void) {
    PLL_Init();                     // Initialize PLL and set system clock
    PE1_Digital_Output_Init();      // Configure PE1 as digital output
    ADC0_Init();                    // Initialize ADC0 (PE3 – AIN0)

    Timer2A_Init_OneShot_us(pulse_width_us);  // Initial one-shot setup
    Timer1A_Init_Periodic_ms(20U);            // 20 ms (50 Hz) periodic frame

    while (1) {
        // All motor control handled in interrupts
    }
}

/* =======================================================
   ---------- GPIO Configuration ----------
   ======================================================= */
void PE1_Digital_Output_Init(void) {
    SYSCTL->RCGCGPIO |= GPIO_PORTE_CLOCK_EN;   // Enable clock for Port E
    for (j = 0; j < 3; j++);                   // Short delay

    GPIOE->AFSEL &= ~GPIO_PORTE_PIN1_OP;       // Disable alternate function on PE1
    GPIOE->DEN   |= GPIO_PORTE_PIN1_EN;        // Enable digital I/O on PE1
    GPIOE->DIR   |= GPIO_PORTE_PIN1_OP;        // Set PE1 as output
    GPIOE->DATA  &= ~GPIO_PORTE_PIN1;          // Set initial output LOW
}

/* =======================================================
   ---------- ADC0 Initialization ----------
   Using PE3 – AIN0 – Potentiometer input
   ======================================================= */
void ADC0_Init(void) {
    SYSCTL->RCGCADC |= 0x01;                   // Enable ADC0 clock
    SYSCTL->RCGCGPIO |= 0x10;                  // Enable Port E clock
    for (j = 0; j < 3; j++);                   // Delay for stabilization

    GPIOE->PDR |= 0x08;                        // Enable weak pull-down on PE3
    GPIOE->AFSEL |= 0x08;                      // Enable alternate function on PE3
    GPIOE->DEN &= ~0x08;                       // Disable digital on PE3
    GPIOE->AMSEL |= 0x08;                      // Enable analog mode on PE3

    ADC0->ACTSS &= ~0x08;                      // Disable SS3 during configuration
    ADC0->EMUX &= ~0xF000;                     // Software trigger for SS3
    ADC0->SSMUX3 = 0;                          // AIN0 (PE3)
    ADC0->SSCTL3 = 0x06;                       // IE0 + END0 bits
    ADC0->ACTSS |= 0x08;                       // Enable SS3
}

/* =======================================================
   ---------- ADC0 Read Function ----------
   ======================================================= */
uint16_t ADC0_Read(void) {
    ADC0->PSSI = 0x08;                         // Start SS3 conversion
    while ((ADC0->RIS & 0x08) == 0);           // Wait for conversion complete
    uint16_t result = ADC0->SSFIFO3 & 0xFFF;   // Read 12-bit result
    ADC0->ISC = 0x08;                          // Clear interrupt
    return result;
}

/* =======================================================
   ---------- Timer2A One-Shot Configuration ----------
   ======================================================= */
void Timer2A_Init_OneShot_us(uint32_t delay_us) {
    SYSCTL->RCGCTIMER |= TIMER2_CLOCK_EN;      // Enable Timer2 clock
    for (j = 0; j < 3; j++);

    TIMER2->CTL  = 0x00;                       // Disable Timer2A during setup
    TIMER2->CFG  = 0x04;                       // 16-bit mode
    TIMER2->TAMR = 0x01;                       // One-shot mode
    TIMER2->TAPR = 16 - 1;                     // 1 µs tick (16MHz / 16)
    TIMER2->TAILR = delay_us ? delay_us - 1 : 0;  // Load timer safely
    TIMER2->ICR   = 0x01;                      // Clear timeout flag
    TIMER2->IMR  |= 0x01;                      // Enable timeout interrupt

    NVIC_SetPriority(TIMER2A_IRQn, 2);
    NVIC_EnableIRQ(TIMER2A_IRQn);
}

/* =======================================================
   ---------- Timer1A Periodic Configuration ----------
   ======================================================= */
void Timer1A_Init_Periodic_ms(uint32_t period_ms) {
    SYSCTL->RCGCTIMER |= TIMER1_CLOCK_EN;
    for (j = 0; j < 3; j++);

    TIMER1->CTL  = 0x00;                       // Disable Timer1A
    TIMER1->CFG  = 0x04;                       // 16-bit mode
    TIMER1->TAMR = 0x02;                       // Periodic mode
    TIMER1->TAPR = 250 - 1;                    // 1 ms tick
    TIMER1->TAILR = (64 * period_ms) - 1;      // 20 ms period
    TIMER1->ICR   = 0x01;                      // Clear flag
    TIMER1->IMR  |= 0x01;                      // Enable interrupt

    NVIC_SetPriority(TIMER1A_IRQn, 1);
    NVIC_EnableIRQ(TIMER1A_IRQn);

    TIMER1->CTL |= 0x01;                       // Start Timer1A
}

/* =======================================================
   ---------- INTERRUPT SERVICE ROUTINES ----------
   ======================================================= */

/* Timer1A ISR:
   - Every 20 ms, read potentiometer
   - If valid reading ? output pulse (1–2 ms)
   - If invalid/disconnected ? stop motor signal
*/
void TIMER1A_Handler(void) {
    static uint8_t invalid_count = 0;  // consecutive invalid readings
    uint16_t adc_value = ADC0_Read();  // Read potentiometer (0–4095)

    if (adc_value < 50 || adc_value > 4045) {
        invalid_count++;
        if (invalid_count >= 3) {
            GPIOE->DATA &= ~GPIO_PORTE_PIN1;  // Keep output LOW (stop motor)
            TIMER2->CTL &= ~0x01;             // Stop one-shot timer
            invalid_count = 3;                // Clamp count
        }
    } else {
        invalid_count = 0;                    // Valid reading resets counter
        pulse_width_us = 1000 + (adc_value * 1000) / 4095;

        TIMER2->TAILR = pulse_width_us - 1;   // Update pulse width
        GPIOE->DATA |= GPIO_PORTE_PIN1;       // Start pulse (PE1 HIGH)
        TIMER2->CTL |= 0x01;                  // Start one-shot timer
    }

    TIMER1->ICR = 0x01;                       // Clear Timer1A flag
}

/* Timer2A ISR:
   - Ends pulse after pulse width expires
*/
void TIMER2A_Handler(void) {
    TIMER2->ICR = 0x01;                        // Clear flag
    GPIOE->DATA &= ~GPIO_PORTE_PIN1;           // End pulse (PE1 LOW)
    TIMER2->CTL &= ~0x01;                      // Stop timer
}

/* =======================================================
   ---------- PLL Initialization ----------
   ======================================================= */
void PLL_Init(void) {
    SYSCTL->RCC2 |= 0x80000000;                // Use RCC2
    SYSCTL->RCC2 |= 0x00000800;                // BYPASS2
    SYSCTL->RCC  = (SYSCTL->RCC & ~0x000007C0) + 0x00000540;
    SYSCTL->RCC2 &= ~0x00000070;               // MOSC source
    SYSCTL->RCC2 &= ~0x00002000;               // Activate PLL
    SYSCTL->RCC2 |= 0x40000000;                // Use 400 MHz PLL
    SYSCTL->RCC2 = (SYSCTL->RCC2 & ~0x1FC00000) + (24 << 22);
    while ((SYSCTL->RIS & 0x00000040) == 0);   // Wait for PLL lock
    SYSCTL->RCC2 &= ~0x00000800;               // Enable PLL
}
