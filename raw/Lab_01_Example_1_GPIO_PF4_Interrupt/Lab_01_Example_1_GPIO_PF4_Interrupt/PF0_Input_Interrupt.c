#include "TM4C123.h"                    // Device header

unsigned volatile long j;

void PLL_Init(void);
void PF3_Digital_Output_Init();
void PF4_External_Interrupt_Input_Init();

int main(){
	PLL_Init();
	PF3_Digital_Output_Init();
	PF4_External_Interrupt_Input_Init();
	while(1){
	}
}

void GPIOF_Handler(){
	if((GPIOF->MIS&0x10)==0x10){		// PF4
		GPIOF->DATA ^= 0x08;
		GPIOF->ICR |= 0x10;
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

void PF3_Digital_Output_Init(){
	// Step 1: Clock enable on PortF
	SYSCTL->RCGCGPIO |= 0x20;
	for (j =0; j < 3 ; j++)		// at least 3 clock cyles
	
	// Step 2: APB is selected for PortF by selecting
	// 0x40025000 as Base Address in DATA section
	
	// Step 3: Disable alternate functionality on PortF
	GPIOF->AFSEL |= 0x00;
	
	// Step 4: Enable digital pin functionaliy on PortF pin 3
	GPIOF->DEN |= 0x08; // Digital enable for PF3
	
	// Step 5: Set PortF pin 3 as an output pin
	GPIOF->DIR |= 0x08; // PF3 as output
}

void PF4_External_Interrupt_Input_Init(){
	// Step 1: Clock enable on PortF
	SYSCTL->RCGCGPIO |= 0x20;
	for (j =0; j < 3 ; j++)		// at least 3 clock cyles
	
	// Step 2: APB is selected for PortF by selecting
	// 0x40025000 as Base Address in DATA section
	
	// Step 3: Disable alternate functionality on PortF
	GPIOF->AFSEL |= 0x00;
	
	// Step 4: Enable digital pin functionaliy on PortF pin 4
	GPIOF->DEN |= 0x10; // Digital enable for PF4
	
	// Step 5: Set PortF pin 4 as an input pin
	GPIOF->DIR &= ~0x10; // PF4 as input
	GPIOF->PUR |= 0x10;		// Use internal weak pull-up resistors
	
	// Step 6: Perform GPIO Port Interrupt Configurations
	GPIOF->IS &= ~0x10;			// Enable edge-triggered interrupt at PF4
	GPIOF->IEV |= 0x10;			// Enable rising edge triggered interrupt at PF4
	GPIOF->IBE &= ~0x10;		// Disable both edge triggered interrupt
	GPIOF->IM |= 0x10;			// PF0 interrupt is not masked
	GPIOF->ICR |= 0x10; 		// Clearing interrupt flags (in RIS & MIS)
	
	// Step 7: NVIC Configurations
	// Set the NVIC Register for Enabling the EXTI for GPIOF
	NVIC->ISER[0] |= (1<<30);			// GPIO PortF IRQ # = 30 (datasheet pg # 105)
	// Set priority for the GPIOF Interrupt
	NVIC->IPR[7] = 0x00600000;	// Priority 3
}