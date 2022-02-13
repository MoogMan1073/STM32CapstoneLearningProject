/*
 * LCD4bitComm.h
 *
 *  Created on: Feb 12, 2022
 *      Author: elimo
 */

#ifndef SRC_LCD_4BITCOMM_H_
#define SRC_LCD_4BITCOMM_H_

#include <stdio.h>

#include "stm32f030x8.h"
#include "stm32f0xx_hal.h"

#define LCD_CMD_DELAY_TIME 1

#define LCD_PIN_COUNT 7
#define LCD_DATA_PIN_COUNT 4

#define NO_OFFSET 0
#define ENABLE_LCD 1
#define DISABLE_LCD 0
#define RS_PIN_CHAR_MODE 1
#define RS_PIN_INS_MODE 0
#define RW_PIN_WRITE_MODE 0
#define RW_PIN_READ_MODE 1

#define LCD_CLR_CMD 0b00000001
#define LCD_SET_CURSOR_CMD 0b10000000
#define LCD_SET_CURSOR_ROW_OFFSET 6
#define LCD_4BIT_MODE_CMD 0b0010
#define LCD_2ROW_MODE_CMD 0b1000
#define LCD_TURN_ON_DISP_CMD 0b00001100
#define LCD_INC_ADDR_BY_1_CMD 0b00000110

#define LCD_INIT_STEP0_DELAY 200
#define LCD_INIT_STEP1_DELAY 100

#define SEND_BYTE_DONE -1
#define NIBBLE_SIZE 4

#define LCD_INT_FORMAT "%d "
#define LCD_FLOAT_FORMAT "%.3f"

class LCD_4bitComm
{
public:
	LCD_4bitComm(
			GPIO_TypeDef *d4_port, uint16_t d4_pin,
			GPIO_TypeDef *d5_port, uint16_t d5_pin,
			GPIO_TypeDef *d6_port, uint16_t d6_pin,
			GPIO_TypeDef *d7_port, uint16_t d7_pin,
			GPIO_TypeDef *en_port, uint16_t en_pin,
			GPIO_TypeDef *rw_port, uint16_t rw_pin,
			GPIO_TypeDef *rs_port, uint16_t rs_pin,
			uint16_t rows, uint16_t cols
											);
	void SendBitToReg(GPIO_TypeDef *port, int pin, char c, int pos);
	void Enable(int timeDelay);
	void SetToWrite();
	void SetLCDToRead();
	void InsMode();
	void CharacterMode();
	void SendByte(char character);
	void SendNibble(char character);
	void SendChar(char c);
	void SendIns(char c);
	void Clear();
	void SetCursor(int ROW, int COL);
	char TestCharCycle(char character, int delay);
	void SendString(char *String);
	void SendInt(int Num, uint8_t len);
	void SendFloat(float Num, uint8_t len);
	void Init(void);

private:
	int _lcd_pins[LCD_PIN_COUNT] = {};
	GPIO_TypeDef* _lcd_pin_ports[LCD_PIN_COUNT] = {};
	int _rs_pin;
	int _rw_pin;
	int _en_pin;
	GPIO_TypeDef* _en_port;
	GPIO_TypeDef* _rw_port;
	GPIO_TypeDef* _rs_port;
	int _rows;
	int _cols;

};
#endif /* SRC_LCD_4BITCOMM_H_ */
