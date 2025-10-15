#include "TM4C123.h"                    // Device header

unsigned volatile long j;

void PLL_Init(void);
void PF2_3_Digital_Output_Init();
void PF0_4_External_Interrupt_Input_Init();

int main(){
	PLL_Init();
	PF2_3_Digital_Output_Init();
	PF0_4_External_Interrupt_Input_Init();
	while(1){
	}
}

void GPIOF_Handler(){
	if((GPIOF->MIS&0x10)==0x10){		// PF4
		GPIOF->DATA |= 0x08;
		GPIOF->DATA &= ~0x04;
		GPIOF->ICR |= 0x10;
	}
	else if((GPIOF->MIS&0x01)==0x01){		// PF0
		GPIOF->DATA |= 0x04;
		GPIOF->DATA &= ~0x08;
		GPIOF->ICR |= 0x01;
	}
}

// Configure the system clock to 16 MHz using PLL-------------------------------
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

void PF2_3_Digital_Output_Init(){
	// Step 1: Clock enable on PortF
	SYSCTL->RCGCGPIO |= 0x20;
	for (j =0; j < 3 ; j++)		// at least 3 clock cyles
	
	// Step 2: APB is selected for PortF by selecting
	// 0x40025000 as Base Address in DATA section
	
	// Step 3: Disable alternate functionality on PortF
	GPIOF->AFSEL |= 0x00;
	
	// Step 4: Enable digital pin functionaliy on PortF pin 2&3
	GPIOF->DEN |= 0x0C; // Digital enable for PF2&3
	
	// Step 5: Set PortF pin 2&3 as an output pin
	GPIOF->DIR |= 0x0C; // PF2&3 as output
}

void PF0_4_External_Interrupt_Input_Init(){
	// Step 1: Clock enable on PortF
	SYSCTL->RCGCGPIO |= 0x20;
	for (j =0; j < 3 ; j++)		// at least 3 clock cyles
	
	// Step 2: APB is selected for PortF by selecting
	// 0x40025000 as Base Address in DATA section
	
	// Unlock the PortF Pin4
	GPIOF->LOCK = 0x4C4F434B;
	GPIOF->CR = 0x01;
	GPIOF->LOCK = 0;
	
	// Step 3: Disable alternate functionality on PortF
	GPIOF->AFSEL |= 0x00;
	
	// Step 4: Enable digital pin functionaliy on PortF pin 0&4
	GPIOF->DEN |= 0x11; // Digital enable for PF0&4
	
	// Step 5: Set PortF pin 0&4 as an input pin
	GPIOF->DIR &= ~0x11; // PF0&4 as input
	GPIOF->PUR |= 0x11;		// Use internal weak pull-up resistors
	
	// Step 6: Perform GPIO Port Interrupt Configurations
	GPIOF->IS &= ~0x11;					// Disable edge triggered interrupt
	GPIOF->IBE &= ~0x11;				// Disable both edge triggered interrupt
	GPIOF->IEV &= ~0x11;				// Disable rising edge triggered interrupt
	GPIOF->IM |= 0x11;					// PF0 & PF4 are not masked
	GPIOF->ICR |= 0x11; 				// Clearing interrupt flags
	
	// Step 7: NVIC Configurations
	// Set the NVIC Register for Enabling the EXTI for GPIOF
	NVIC->ISER[0] |= (1<<30);			// GPIO PortF IRQ # = 30
	// Set priority for the GPIOF Interrupt
	NVIC->IPR[7] = 0x00600000;	// Priority 3
}
