#include "TM4C123.h"             // Device header

#define GPIO_PORTF_CLOCK_EN 0x20 // Port F clock gating control  (b 0010 0000)
#define GPIO_PORTF_AFSEL_DS 0x00 // Port F alternate function disable
#define GPIO_PORTF_PIN3_EN 0x08  // Port F pin 3 digital enable
#define GPIO_PORTF_PIN3_OP 0x08  // Port F pin 3 as digital output
#define GPIO_PORTF_PIN3 0x08 		 // Mask for pin 3 write operation (0000 1000)

void PF3_Digital_Output_Init();
void timer0A_OneShot(uint32_t mDelay);

unsigned volatile long j;

int main(){
	// Initialize PortF pin 3 as digital out
	PF3_Digital_Output_Init();
	
	while(1){
		GPIOF->DATA ^= GPIO_PORTF_PIN3;		// LED Toggle
		timer0A_OneShot(4);		// 4 mili-second delay
	}
}

void PF3_Digital_Output_Init(){
	// Step 1: Clock enable on PortF
	SYSCTL->RCGCGPIO |= GPIO_PORTF_CLOCK_EN;
	for (j =0; j < 3 ; j++)		// at least 3 clock cyles
	
	// Step 2: APB is selected for PortF by selecting
	// 0x40025000 as Base Address in DATA section
	
	// Step 3: Disable alternate functionality on PortF
	GPIOF->AFSEL |= GPIO_PORTF_AFSEL_DS;
	
	// Step 4: Enable digital pin functionaliy on PortF pin 3
	GPIOF->DEN |= GPIO_PORTF_PIN3_EN; // Digital enable for PF3
	
	// Step 5: Set PortF pin 3 as an output pin
	GPIOF->DIR |= GPIO_PORTF_PIN3_OP; // PF3 as output
}

void timer0A_OneShot(uint32_t mDelay){
	
	// Step 1: Enable Timer Clock on timer0
	SYSCTL->RCGCTIMER |= 0x01;		// b 0000 0001
	for (j =0; j < 3 ; j++)		// at least 3 clock cyles
	
	// Step 2: Ensure Timer is disabled before making any changes
	TIMER0->CTL = 0x00;					// TAEN = 0, i.e., timer0A is disablled
	
	// Step 3: Select Mode of Operation of timer0 (Split/cancatenated/RTC)
	TIMER0->CFG = 0x04;					// timer0 is used as a 16-bit (split) timer, i.e., timer0A
	TIMER0->TAMR = 0x01; 				// TAMR = 1 (1-shot), TACDIR = 0 (count-down)
	
	// Step 4: Load counter start value in Interval Load Register
	TIMER0->TAILR = 16000*mDelay - 1;		// for 16 MHz, 16000 will generate 1 mSec delay

	// Step 5: Interrupt configurations
	TIMER0->IMR = 0x00;					// No interrupt enabled (RO Register)
	TIMER0->ICR = 0x01;					// Clear timer status flag (TATORIS, TATOMIS)
	
	// Step 6: Enable the Timer and start counting
	TIMER0->CTL |= 0x01;					// TAEN = 1
	
	// Step 7: Poll TATORIS (Time-Out Raw Interrupt Status) bit to check timer0A timeout
	while((TIMER0->RIS & 0x01)==0); // Wait for timeout flag to set
	
	// After time-out, timer stops automatically in one-shot mode
	TIMER0->ICR = 0x01;					// Clear timer status flag (TATORIS, TATOMIS)
}
