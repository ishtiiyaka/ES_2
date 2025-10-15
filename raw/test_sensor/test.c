//**************************************************************
// TCRT5000 Edge Detection Test for TM4C123GH6PM
// PF4 = Sensor DO input
// PF1 = Red LED (blinks once per detection)
//**************************************************************

#include <stdint.h>
#include "tm4c123gh6pm.h"

void PortF_Init(void);
void DelayMs(int n);

int main(void) {
    PortF_Init();

    int lastState = GPIO_PORTF_DATA_R & 0x10;
    int currentState;

    while (1) {
        currentState = GPIO_PORTF_DATA_R & 0x10;

        // Detect falling edge: HIGH ? LOW
        if (lastState == 0x10 && currentState == 0x00) {
            GPIO_PORTF_DATA_R |= 0x02;   // LED ON
            DelayMs(100);                // Keep LED on briefly
            GPIO_PORTF_DATA_R &= ~0x02;  // LED OFF
        }

        lastState = currentState;
    }
}

//**************************************************************
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x20;
    while((SYSCTL_PRGPIO_R & 0x20) == 0);

    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R = 0x1F;

    GPIO_PORTF_DIR_R |= 0x02;   // PF1 = Output (LED)
    GPIO_PORTF_DIR_R &= ~0x10;  // PF4 = Input (sensor)
    GPIO_PORTF_DEN_R |= 0x12;   // Digital enable PF1, PF4
    GPIO_PORTF_PUR_R |= 0x10;   // Pull-up on PF4
}

//**************************************************************
void DelayMs(int n) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < 3180; j++) { }   // 1 ms delay
    }
}
