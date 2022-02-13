/*
 * I2CFunctions.h
 *
 *  Created on: Feb 5, 2022
 *      Author: elimo
 */

#ifndef INC_I2CFUNCTIONS_H_
#define INC_I2CFUNCTIONS_H_

void I2C1Setup(void)
{
	// I2C1 uses port B pines 6 and 7
	  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	  // Set alt. funct. for Port B pins 6 & 7
	  GPIOB->MODER &= ~GPIO_MODER_MODER6_0;
	  GPIOB->MODER |= GPIO_MODER_MODER6_1;
	  GPIOB->MODER &= ~GPIO_MODER_MODER7_0;
	  GPIOB->MODER |= GPIO_MODER_MODER7_1;
	  // Open drain type
	  GPIOB->OTYPER |= GPIO_OTYPER_OT_6;
	  GPIOB->OTYPER |= GPIO_OTYPER_OT_6;
	  GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR6_0;
	  GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR7_0;
	  // Disable internal pullups, use external resistors in circuit
	  GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR6;
	  GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR7;
	  // I2C Alt/ Func. Setup
	  GPIOB->AFR[0] |= (GPIO_AF1_I2C1 << 28) | (GPIO_AF1_I2C1 << 24);
}

void I2CInit(void)
{
	  // En & config I2C1
	  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	  I2C1->TIMINGR = (uint32_t)0x0010020A; //Use STM32 I2C Timing Excel sheet to calculate
	  I2C1->CR1 |= I2C_CR1_PE;
}

void SetSlaveAddr(uint8_t Slave_Addr)
{
	  // Left shift by 1, bit 0 is for R/W!
	I2C1->CR2 |= (Slave_Addr << 1); // send addr of slave device to cont reg 2
}

void I2C_Start(int NumBytes)
{
	  I2C1->CR2 &= ~(0b11111111 << 16); // Reset # of bytes to read to 0 before declaring # number of bytes to actually send
	  I2C1->CR2 |= (NumBytes << 16); // Shift the number of bytes to read by 16

	  // Start the communication
	  I2C1->CR2 |= I2C_CR2_START; // Est start condition
	  while(I2C1->CR2 & I2C_CR2_START); // verify start condition is true by waiting until the start bit cleared by HW
}

void I2C_Stop(void)
{
	  I2C1->CR2 |= I2C_CR2_STOP; // Est Stop Condition
	  while(I2C1->CR2 & I2C_CR2_STOP); // verify stop condition is true by waiting until the start bit cleared by HW

}

void I2C_StartWrite(int NumBytestoWrite)
{
	  I2C1->CR2 &= ~I2C_CR2_RD_WRN; // Begin a write CMD
	  I2C_Start(NumBytestoWrite);
}

void I2C_TransmitByte(uint8_t TransmitByte)
{
	  I2C1->TXDR = TransmitByte; // Send reg
	  while(!(I2C1->ISR & I2C_ISR_TXE));  // wait for transmit buffer to empty
}

void I2C_StartRead(int NumBytestoRead)
{
	I2C1->CR2 |= I2C_CR2_RD_WRN; // Begin a read CMD
	I2C_Start(NumBytestoRead);
}

uint8_t I2C_Read_Byte(void)
{
	  I2C_StartRead(1);
	  while(!(I2C1->ISR & I2C_ISR_RXNE)); // wait for receive buffer to empty

	  return I2C1->RXDR;
}

uint8_t I2C_Read_One_Register(uint8_t dev_addr, uint8_t reg_addr)
{
	  // ----- Read Seq. -------//
	  // 1. Specify addr of device addr and how much data to read
	  SetSlaveAddr(dev_addr);
	  I2C_StartWrite(1);

	  // 2. Specify register to read from
	  I2C_TransmitByte(reg_addr);

	  // 3. Read data in the register(s)
	  uint8_t received_data = I2C_Read_Byte();

	  // Stop the communication
	  I2C_Stop();

	  return received_data;
}

#endif /* INC_I2CFUNCTIONS_H_ */
