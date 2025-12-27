/* Example_LED_Intensity_Variation_PF1.uvprojx
-----------------------------------------------------------------------
This C language program controls the brightness of the on-board user RED
LED of TIVA C Series LaunchPad Evaluation Kit by Texas Instrument by
using PWM Module 1 Channel 5 available at PortF Pin 1.
Note: Microcontroller operating frequency is set to 16 MHz using PLL.
-----------------------------------------------------------------------
Adapted from original PF2 (M1PWM6) example
Originally written by Shujat Ali on 28-Nov-2023.
Compiler and IDE used: Keil uVision 5.34
-----------------------------------------------------------------------
*/

#include "TM4C123.h"                    // Device header

/* Global variables */
volatile unsigned long j;
volatile uint32_t i;

/* Function prototypes */
void PLL_Init(void);
void PF1_as_M1PWM5_Init(void);
void PWM_Module1_Channel5_Init(void);

/* Simple software delay */
void clocks_delay(volatile uint32_t clocks){
	while(clocks--);
}

/* ============================== MAIN ============================== */
int main(void){

	PLL_Init();                    // Configure system clock to 16 MHz
	PF1_as_M1PWM5_Init();          // Configure PF1 as PWM output
	PWM_Module1_Channel5_Init();   // Initialize PWM Module 1 Channel 5

	while(1){
		/* Gradually increase LED brightness */
		for(i = 0; i < 1599; i++){
			PWM1->_2_CMPB = i;      // Increase duty cycle
			clocks_delay(2000);
		}
		clocks_delay(500000);

		/* Gradually decrease LED brightness */
		for(i = 1599; i > 0; i--){
			PWM1->_2_CMPB = i;      // Decrease duty cycle
			clocks_delay(2000);
		}
		clocks_delay(500000);
	}
}

/* ======================= PLL INITIALIZATION ======================= */
/* Configure the system clock to 16 MHz using PLL */
void PLL_Init(void){

  // 0) Use RCC2 register
  SYSCTL->RCC2 |=  0x80000000;   // USERCC2

  // 1) Bypass PLL while initializing
  SYSCTL->RCC2 |=  0x00000800;   // BYPASS2

  // 2) Select crystal value and oscillator source
  SYSCTL->RCC  = (SYSCTL->RCC & ~0x000007C0) + 0x00000540; // 16 MHz crystal
  SYSCTL->RCC2 &= ~0x00000070;   // Main oscillator

  // 3) Activate PLL by clearing PWRDN
  SYSCTL->RCC2 &= ~0x00002000;

  // 4) Set system divider
  SYSCTL->RCC2 |= 0x40000000;    // Use 400 MHz PLL
  SYSCTL->RCC2 = (SYSCTL->RCC2 & ~0x1FC00000) + (24 << 22); // 16 MHz

  // 5) Wait for PLL lock
  while((SYSCTL->RIS & 0x00000040) == 0){}

  // 6) Enable PLL by clearing BYPASS
  SYSCTL->RCC2 &= ~0x00000800;
}

/* ======================= PF1 PWM PIN INIT ========================= */
/* Configure PortF Pin 1 as M1PWM5 */
void PF1_as_M1PWM5_Init(void){

	// Step 1: Enable clock for PortF
	SYSCTL->RCGCGPIO |= 0x20;      // Enable PortF clock
	for(j = 0; j < 3; j++);        // Wait for clock to stabilize

	// Step 2: Enable alternate function on PF1
	GPIOF->AFSEL |= 0x02;          // PF1 alternate function

	// Step 3: Enable digital functionality on PF1
	GPIOF->DEN |= 0x02;

	// Step 4: Set PF1 as output pin
	GPIOF->DIR |= 0x02;

	// Step 5: Configure PF1 as M1PWM5
	// (Table 10-2 of datasheet)
	GPIOF->PCTL &= 0xFFFFFF0F;     // Clear PMC1 field
	GPIOF->PCTL |= 0x00000050;     // Assign M1PWM5
}

/* ======================= PWM INITIALIZATION ======================= */
/* Initialize PWM Module 1 Channel 5 (Generator 2, Output B) */
void PWM_Module1_Channel5_Init(void){

	// Step 1: Enable clock for PWM Module 1
	SYSCTL->RCGCPWM |= (1 << 1);
	for(j = 0; j < 3; j++);        // Wait for clock

	// Disable PWM clock divider
	SYSCTL->RCC &= ~(1 << 20);

	// Step 2: Disable Generator 2 before configuration
	PWM1->_2_CTL = 0x00;

	// Step 3: Set LOAD value for 10 kHz PWM
	// (16 MHz / 10 kHz = 1600)
	PWM1->_2_LOAD = 1600;

	// Step 4: Set initial duty cycle to 50%
	PWM1->_2_CMPB = 800;

	// Step 5: Configure Generator B
	// PWM HIGH on reload, LOW on compare match
	PWM1->_2_GENB = 0x0000080C;

	// Step 6: Enable Generator 2
	PWM1->_2_CTL |= 0x01;

	// Step 7: Enable PWM output M1PWM5
	PWM1->ENABLE |= 0x20;
}
