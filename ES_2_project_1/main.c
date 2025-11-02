#include "TM4C123.h"
#include <stdint.h>
#include <stdio.h>

/* ---------- Clock Enables ---------- */
#define GPIO_PORTA_CLOCK_EN   0x01
#define GPIO_PORTB_CLOCK_EN   0x02
#define UART0_CLOCK_EN        0x01
#define I2C0_CLOCK_EN         0x01

/* ---------- MPU6050 Address ---------- */
#define MPU6050_ADDR          0x68

/* ---------- Function Prototypes ---------- */
void PLL_Init(void);
void UART0_Init(void);
void I2C0_Init(void);
void I2C0_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C0_Read(uint8_t slaveAddr, uint8_t regAddr);
void UART0_Tx(char c);
void UART0_Print(char *str);
void delay_ms(uint32_t n);

/* ---------- Main Function ---------- */
int main(void){
    PLL_Init();

    SYSCTL->RCGCGPIO |= (GPIO_PORTA_CLOCK_EN | GPIO_PORTB_CLOCK_EN);
    SYSCTL->RCGCUART |= UART0_CLOCK_EN;
    SYSCTL->RCGCI2C  |= I2C0_CLOCK_EN;
    delay_ms(10);

    UART0_Init();
    I2C0_Init();

    // ---------- Initialize MPU6050 ----------
    I2C0_Write(MPU6050_ADDR, 0x6B, 0x00);  // Wake up
    I2C0_Write(MPU6050_ADDR, 0x1C, 0x00);  // ±2g range
    I2C0_Write(MPU6050_ADDR, 0x1B, 0x00);  // ±250°/s gyro
    I2C0_Write(MPU6050_ADDR, 0x19, 0x07);  // Sample rate divider
    delay_ms(100);

    UART0_Print("MPU6050 Initialized\r\n");

    while(1){
        int16_t accelX, accelY, accelZ;
        char buffer[64];

        accelX = (I2C0_Read(MPU6050_ADDR, 0x3B) << 8) | I2C0_Read(MPU6050_ADDR, 0x3C);
        accelY = (I2C0_Read(MPU6050_ADDR, 0x3D) << 8) | I2C0_Read(MPU6050_ADDR, 0x3E);
        accelZ = (I2C0_Read(MPU6050_ADDR, 0x3F) << 8) | I2C0_Read(MPU6050_ADDR, 0x40);

        sprintf(buffer, "%d,%d,%d\r\n", accelX, accelY, accelZ);
        UART0_Print(buffer);

        delay_ms(200);
    }
}

/* ---------- I2C0 Initialization ---------- */
void I2C0_Init(void){
    SYSCTL->RCGCGPIO |= GPIO_PORTB_CLOCK_EN;
    while((SYSCTL->PRGPIO & GPIO_PORTB_CLOCK_EN) == 0);

    GPIOB->AFSEL |= 0x0C;              // PB2=SCL, PB3=SDA
    GPIOB->ODR   |= 0x08;              // Open-drain for SDA
    GPIOB->DEN   |= 0x0C;              // Digital enable
    GPIOB->PCTL   = (GPIOB->PCTL & 0xFFFF00FF) | 0x00003300;

    SYSCTL->RCGCI2C |= I2C0_CLOCK_EN;
    while((SYSCTL->PRI2C & I2C0_CLOCK_EN) == 0);

    I2C0->MCR = 0x10;                  // Master mode
    I2C0->MTPR = 0x07;                 // 100 kbps
}

/* ---------- I2C0 Write ---------- */
void I2C0_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data){
    I2C0->MSA = (slaveAddr << 1);
    I2C0->MDR = regAddr;
    I2C0->MCS = 0x03;
    while(I2C0->MCS & 0x01);

    I2C0->MDR = data;
    I2C0->MCS = 0x05;
    while(I2C0->MCS & 0x01);
}

/* ---------- I2C0 Read ---------- */
uint8_t I2C0_Read(uint8_t slaveAddr, uint8_t regAddr){
    uint8_t data;
    I2C0->MSA = (slaveAddr << 1);
    I2C0->MDR = regAddr;
    I2C0->MCS = 0x03;
    while(I2C0->MCS & 0x01);

    I2C0->MSA = (slaveAddr << 1) | 1;
    I2C0->MCS = 0x07;
    while(I2C0->MCS & 0x01);
    data = I2C0->MDR;

    return data;
}

/* ---------- UART0 Initialization ---------- */
void UART0_Init(void){
    SYSCTL->RCGCGPIO |= GPIO_PORTA_CLOCK_EN;
    while((SYSCTL->PRGPIO & GPIO_PORTA_CLOCK_EN) == 0);

    GPIOA->AFSEL |= 0x03;
    GPIOA->DEN   |= 0x03;
    GPIOA->PCTL   = (GPIOA->PCTL & 0xFFFFFF00) | 0x00000011;

    SYSCTL->RCGCUART |= UART0_CLOCK_EN;
    while((SYSCTL->PRUART & UART0_CLOCK_EN) == 0);

    UART0->CTL &= ~0x01;
    UART0->IBRD = 104;    // 9600 baud @ 16 MHz
    UART0->FBRD = 11;
    UART0->LCRH = 0x60;
    UART0->CTL = 0x301;
}

/* ---------- UART Send ---------- */
void UART0_Tx(char c){
    while(UART0->FR & 0x20);
    UART0->DR = c;
}

void UART0_Print(char *str){
    while(*str){
        UART0_Tx(*str++);
    }
}

/* ---------- Delay ---------- */
void delay_ms(uint32_t n){
    volatile uint32_t i;
    while(n--){
        for(i = 0; i < 16000; i++);
    }
}

/* ---------- PLL Initialization ---------- */
void PLL_Init(void){
    SYSCTL->RCC2 |= 0x80000000;
    SYSCTL->RCC2 |= 0x00000800;
    SYSCTL->RCC  = (SYSCTL->RCC &~0x000007C0) + 0x00000540;
    SYSCTL->RCC2 &= ~0x00000070;
    SYSCTL->RCC2 &= ~0x00002000;
    SYSCTL->RCC2 |= 0x40000000;
    SYSCTL->RCC2 = (SYSCTL->RCC2 &~0x1FC00000) + (24<<22);
    while((SYSCTL->RIS & 0x00000040)==0){};
    SYSCTL->RCC2 &= ~0x00000800;
}
