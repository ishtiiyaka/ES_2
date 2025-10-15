#include <stdint.h>
#include "tm4c123gh6pm.h"

void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x20;         // Enable clock for Port F
    GPIO_PORTF_DIR_R |= 0x08;          // PF3 as output (Green LED)
    GPIO_PORTF_DEN_R |= 0x08;          // Digital enable PF3
}

void Timer3A_Init(void) {
    SYSCTL_RCGCTIMER_R |= 0x08;        // Enable clock for Timer3
    TIMER3_CTL_R = 0;                  // Disable Timer3A during setup
    TIMER3_CFG_R = 0x00;               // 32-bit timer configuration (concatenated)
    TIMER3_TAMR_R = 0x01;              // One-shot mode
    TIMER3_TAILR_R = 16000000 - 1;     // 1 second delay at 16 MHz
    TIMER3_ICR_R = 0x01;               // Clear timeout flag
    TIMER3_CTL_R |= 0x01;              // Enable Timer3A
}

int main(void) {
    PortF_Init();

    while (1) {
        Timer3A_Init();                // Start one-shot timer
        while ((TIMER3_RIS_R & 0x01) == 0); // Wait for timeout
        GPIO_PORTF_DATA_R ^= 0x08;     // Toggle Green LED
    }
}