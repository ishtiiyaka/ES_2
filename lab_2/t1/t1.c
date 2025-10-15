#include <stdint.h>
#include "tm4c123gh6pm.h"

#define GREEN_LED 0x08

void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x20;         // Enable clock for Port F
    while ((SYSCTL_PRGPIO_R & 0x20) == 0); // Wait until Port F is ready
    GPIO_PORTF_DIR_R |= GREEN_LED;     // PF3 as output
    GPIO_PORTF_DEN_R |= GREEN_LED;     // Digital enable PF3
}

void Timer3A_Init(void) {
    SYSCTL_RCGCTIMER_R |= 0x08;        // Enable clock for Timer3
    while ((SYSCTL_PRTIMER_R & 0x08) == 0); // Wait until Timer3 is ready

    TIMER3_CTL_R &= ~0x01;             // Disable Timer3A during setup
    TIMER3_CFG_R = 0x00;               // Use 32-bit timer (concatenated mode)
    TIMER3_TAMR_R = 0x01;              // One-shot mode
    TIMER3_TAILR_R = 16000000 - 1;     // 1 second delay (assuming 16 MHz clock)
    TIMER3_ICR_R = 0x01;               // Clear timeout flag
    TIMER3_CTL_R |= 0x01;              // Enable Timer3A
}

int main(void) {
    PortF_Init();

    while (1) {
        Timer3A_Init();                        // Start one-shot timer
        while ((TIMER3_RIS_R & 0x01) == 0);    // Wait for timeout
        GPIO_PORTF_DATA_R ^= GREEN_LED;        // Toggle Green LED
    }
}