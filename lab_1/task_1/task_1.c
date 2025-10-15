#include <stdint.h>
#include "tm4c123gh6pm.h"

void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x20;       // Enable clock for Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B;  // Unlock PF0
    GPIO_PORTF_CR_R |= 0x10;         // Allow changes to PF4
    GPIO_PORTF_DIR_R &= ~0x10;       // PF4 as input
    GPIO_PORTF_DEN_R |= 0x10;        // Digital enable PF4
    GPIO_PORTF_PUR_R |= 0x10;        // Enable pull-up resistor

    GPIO_PORTF_IS_R &= ~0x10;        // Edge-sensitive
    GPIO_PORTF_IBE_R |= 0x10;        // Both edges
    GPIO_PORTF_ICR_R |= 0x10;        // Clear interrupt
    GPIO_PORTF_IM_R |= 0x10;         // Unmask interrupt

    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00100000; // Priority 1
    NVIC_EN0_R |= 0x40000000;        // Enable IRQ30 (Port F)
}

void GPIOPortF_Handler(void) {
    if (GPIO_PORTF_RIS_R & 0x10) {
        if (GPIO_PORTF_DATA_R & 0x10)
            GPIO_PORTF_DATA_R |= 0x08;  // Rising edge ? Green ON
        else
            GPIO_PORTF_DATA_R &= ~0x08; // Falling edge ? Green OFF
        GPIO_PORTF_ICR_R |= 0x10;       // Clear interrupt
    }
}