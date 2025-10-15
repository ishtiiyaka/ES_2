#include "TM4C123.h"             // Device header

#define GPIO_PORTF_CLOCK_EN 0x20 // Port F clock gating control  (b 0010 0000)
#define GPIO_PORTF_AFSEL_DS 0x00 // Port F alternate function disable
#define GPIO_PORTF_PIN3_EN 0x08  // Port F pin 3 digital enable
#define GPIO_PORTF_PIN3_OP 0x08  // Port F pin 3 as digital output
#define GPIO_PORTF_PIN3 0x08 		 // Mask for pin 3 write operation (0000 1000)

void PF3_Digital_Output_Init(void);
void timer0A_Prescaler(uint32_t sec);
void PLL_Init(void);

unsigned volatile long j;

int main(){
	PLL_Init();
	// Initialize PortF pin 3 as digital out
	PF3_Digital_Output_Init();
	
	while(1){
		GPIOF->DATA ^= GPIO_PORTF_PIN3;		// LED Toggle
		timer0A_Prescaler(1);							// 2 seconds delay
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

void timer0A_Prescaler(uint32_t sec){
	
	// Step 1: Enable Timer Clock on timer0
	SYSCTL->RCGCTIMER |= 0x01;		// b 0000 0001
	for (j =0; j < 3 ; j++)		// at least 3 clock cyles
	
	// Step 2: Ensure Timer is disabled before making any changes
	TIMER0->CTL = 0x00;					// TAEN = 0, i.e., timer0A is disablled
	
	// Step 3: Select Mode of Operation of timer0 (Split/cancatenated/RTC)
	TIMER0->CFG = 0x04;					// timer0 is used as a 16-bit (split) timer, i.e., timer0A
	TIMER0->TAMR = 0x02; 				// TAMR = 2 (periodic), TACDIR = 0 (count-down)
	
	// Step 4: Load counter start value 
	TIMER0->TAPR = 250-1;					// 16000000 Hz/250 = 64000 Hz
	TIMER0->TAILR = 32000-1;			// 64000/32000 = 2 Hz (0.5 second)

	// Step 5: Interrupt configurations
	TIMER0->ICR = 0x01;					// Clear timer status flag (TATORIS, TATOMIS)
	
	// Step 6: Enable the Timer and start counting
	TIMER0->CTL |= 0x01;					// TAEN = 1
	
	// One iteration of this loop generates 1 mili-second delay
	for(uint32_t i = 0; i<sec; i++){
		// Step 7: Poll TATORIS (Time-Out Raw Interrupt Status) bit to check timer0A timeout
		while((TIMER0->RIS & 0x01)==0); // Wait for timeout flag to set
		
		// After time-out, timer relaoads automatically and start counting
		// Clear timer status flag i.e., TATORIS flag
		TIMER0->ICR = 0x01;
	}
	// Disable Timer 
	TIMER0->CTL = 0x00;			// TAEN = 0, i.e., timer0A is disablled
}

// Configure the system clock to 16 MHz using PLL
void PLL_Init(void){

  // 0) Use RCC2
  SYSCTL->RCC2 |=  0x80000000;  // USERCC2

  // 1) bypass PLL while initializing
  SYSCTL->RCC2 |=  0x00000800;  // BYPASS2, PLL bypass

  // 2) select the crystal value and oscillator source
  SYSCTL->RCC = (SYSCTL->RCC &~ 0x000007C0) + 0x00000540;  // clear XTAL field, bits 10-6  // 10101, configure for 16 MHz crystal
  SYSCTL->RCC2 &= ~0x00000070;  // configure for main oscillator source

  // 3) activate PLL by clearing PWRDN
  SYSCTL->RCC2 &= ~0x00002000;

  // 4) set the desired system divider
  SYSCTL->RCC2 |= 0x40000000;   // use 400 MHz PLL
  SYSCTL->RCC2 = (SYSCTL->RCC2 &~ 0x1FC00000) + (24<<22);      // configure for 16 MHz clock / / clear system clock divider +

  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL->RIS&0x00000040)==0){}  // wait for PLLRIS bit

  // 6) enable use of PLL by clearing BYPASS
  SYSCTL->RCC2 &= ~0x00000800;
}