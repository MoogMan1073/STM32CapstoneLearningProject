/*
 * LCD4bitComm.cpp
 *
 *  Created on: Feb 12, 2022
 *      Author: elimo
 *
 *	This class implements LCD screens that use an HD44780U driver.
 *	The class only supports up to 2-row screen in 4-bit mode
 *	at the the time of creation.
 */

#include "LCD_4bitComm.h"

/**
 * @brief
 * @param d4_port 	GPIO port connected to the D4 pin on the LCD
 * @param d4_pin	GPIO pin connected to the D4 pin on the LCD
 * @param d5_port 	GPIO port connected to the D5 pin on the LCD
 * @param d5_pin	GPIO pin connected to the D5 pin on the LCD
 * @param d6_port 	GPIO port connected to the D6 pin on the LCD
 * @param d6_pin	GPIO pin connected to the D6 pin on the LCD
 * @param d7_port 	GPIO port connected to the D7 pin on the LCD
 * @param d7_pin	GPIO pin connected to the D7 pin on the LCD
 * @param en_port 	GPIO port connected to the enable pin on the LCD
 * @param en_pin	GPIO pin connected to the enable pin on the LCD
 * @param rw_port 	GPIO port connected to the read/write pin on the LCD
 * @param rw_pin	GPIO pin connected to the read/write pin on the LCD
 * @param rs_port 	GPIO port connected to the reset pin on the LCD
 * @param rs_pin 	GPIO pin connected to the reset pin on the LCD
 * @param rows 		number of rows on the LCD screen
 * @param cols 		number of columns on the LCD screen
 */
LCD_4bitComm::LCD_4bitComm(
		GPIO_TypeDef *d4_port, uint16_t d4_pin,
		GPIO_TypeDef *d5_port, uint16_t d5_pin,
		GPIO_TypeDef *d6_port, uint16_t d6_pin,
		GPIO_TypeDef *d7_port, uint16_t d7_pin,
		GPIO_TypeDef *en_port, uint16_t en_pin,
		GPIO_TypeDef *rw_port, uint16_t rw_pin,
		GPIO_TypeDef *rs_port, uint16_t rs_pin,
		uint16_t rows, uint16_t cols
										)
{
	// Put the enable, read/write, and reset pins into
	// their own private vars for easy accessibility in various functions.
	_en_pin = en_pin;
	_en_port = en_port;
	_rw_pin = rw_pin;
	_rw_port = rw_port;
	_rs_pin = rs_pin;
	_rs_port = rs_port;

	// assign all pins connected to the LCD to an array to allow for
	// for-looping through pins
	_lcd_pins[0] = d4_pin;
	_lcd_pins[1] = d5_pin;
	_lcd_pins[2] = d6_pin;
	_lcd_pins[3] = d7_pin;
	_lcd_pins[4] = _en_pin;
	_lcd_pins[5] = _rw_pin;
	_lcd_pins[6] = _rs_pin;

	// assign all pin ports connected to the LCD to an array to allow for
	// for-looping through pins
	_lcd_pin_ports[0] = d4_port;
	_lcd_pin_ports[1] = d5_port;
	_lcd_pin_ports[2] = d6_port;
	_lcd_pin_ports[3] = d7_port;
	_lcd_pin_ports[4] = _en_port;
	_lcd_pin_ports[5] = _rw_port;
	_lcd_pin_ports[6] = _rs_port;

	// define the rows and columns on the
	// lcd screen used
	_rows = rows;
	_cols = cols;
}

/**
 * @brief Send a bit to out of a particular pin on the microcontroller.
 * @param port	Port of the pin to send the bit out of
 * @param pin	Pin to send the bit out of
 * @param c		Full character to be sent to the LCD
 * @param pos	The particular position of the bit in the character to be sent out of the GPIO pin
 */
void LCD_4bitComm::SendBitToReg(GPIO_TypeDef *port, int pin, char c, int pos)
{
	int result = c >> pos;
	if(result &= 1)
		port->BSRR |= 1 << pin;
	else
		port->BRR |= 1 << pin;
	return;
}

/**
 * @brief Set the Enable pin of the LCD high
 * @param timeDelay delay after enable pin is set high. Some time is required before
 * moving on in the timing sequence, according to the HD44780 datasheet.
 */
void LCD_4bitComm::Enable(int timeDelay)
{
	HAL_Delay(timeDelay);
	SendBitToReg(_en_port, _en_pin, ENABLE_LCD, NO_OFFSET);

}

/**
 * @brief Set the LCD to write mode.
 */
void LCD_4bitComm::SetToWrite()
{
	SendBitToReg(_rw_port, _rw_pin, RW_PIN_WRITE_MODE, NO_OFFSET);
}

/**
 * @brief Set the LCD to read mode.
 */
void LCD_4bitComm::SetLCDToRead()
{
	SendBitToReg(_rw_port, _rw_pin, RW_PIN_READ_MODE, NO_OFFSET);
	Enable(LCD_CMD_DELAY_TIME);
}

/**
 * @brief Sets the LCD to instruction mode. In this mode, the LCD recieves commands
 * as opposed to displaying information on the LCD.
 */
void LCD_4bitComm::InsMode()
{
	SendBitToReg(_rs_port, _rs_pin, RS_PIN_INS_MODE, NO_OFFSET);
}

/**
 * @brief Sets the LCD to Character mode. In this mode, the LCD will display
 * the data sent as opposed to sending commands to the LCD.
 */
void LCD_4bitComm::CharacterMode()
{
	SendBitToReg(_rs_port, _rs_pin, RS_PIN_CHAR_MODE, NO_OFFSET);
}

/**
 * @brief Send a byte to the LCD
 * @param character data to send to the LCD
 */
void LCD_4bitComm::SendByte(char character)
{
	for(int idx = 1; idx > SEND_BYTE_DONE; idx--)
	{
		for(int i = 0; i < LCD_DATA_PIN_COUNT; i++)
		{
			SendBitToReg(_lcd_pin_ports[i], _lcd_pins[i], character, i+(NIBBLE_SIZE*idx));
		}
		HAL_Delay(LCD_CMD_DELAY_TIME); // Disable LCD wait time
		SendBitToReg(_en_port, _en_pin, DISABLE_LCD, NO_OFFSET);
		Enable(LCD_CMD_DELAY_TIME);
	}
}

/**
 * @brief Send a 4 bit instruction to LCD
 * @note This function will only read the most significant
 * 		 4-bits of the data sent into this function
 * @param character data to send LCD
 */
void LCD_4bitComm::SendNibble(char character)
{
	  for(int i = 0; i < LCD_DATA_PIN_COUNT; i++)
	  {
		  SendBitToReg(_lcd_pin_ports[i], _lcd_pins[i], character, i+NIBBLE_SIZE);
	  }
	  HAL_Delay(LCD_CMD_DELAY_TIME); // Disable LCD wait time
	  SendBitToReg(_en_port, _en_pin, DISABLE_LCD, NO_OFFSET);

}

/**
 * @brief Send a character to the LCD
 * @param c	Character to be sent to the LCD
 */
void LCD_4bitComm::SendChar(char c)
{
	SetToWrite();
	CharacterMode();
	Enable(LCD_CMD_DELAY_TIME);
	SendByte(c);
}

/**
 * @brief Send an instruction to the LCD screen.
 * @param c	Instruction to be sent to the LCD screen.
 */
void LCD_4bitComm::SendIns(char c)
{
	SetToWrite();
	InsMode();
	Enable(LCD_CMD_DELAY_TIME);
	SendByte(c);
}

/**
 * @brief Clear the contents of the LCD screen.
 */
void LCD_4bitComm::Clear()
{
	SendIns(LCD_CLR_CMD); // clear LCD
}

/**
 * @brief Set the position of the cursor on the LCD screen.
 * @param ROW Row position of the cursor location.
 * @param COL Column position of the cursor location.
 */
void LCD_4bitComm::SetCursor(int ROW, int COL)
{
	int Instruction = LCD_SET_CURSOR_CMD; // CMD to change cursor location
	Instruction += ((ROW << LCD_SET_CURSOR_ROW_OFFSET) + COL);
	SendIns(Instruction);
}

/**
 * @brief Test sequence to verify functionality of the LCD screen. This sequence will
 * sequentially show characters on the screen starting with the character sent and populate
 * the entire LCD. This function will return the character the sequence ended on which can be
 * used to continue cycling through all of the characters the display can show by default.
 * @param 	character	Character to start the cycling on
 * @param 	delay Delay time in between placing new characters on the display
 * @return	The next character in the cycling sequence, call this function again with this return
 * value to continue the screen cycle.
 */
char LCD_4bitComm::TestCharCycle(char character, int delay)
{
	int count = 0;

	while(count < (_rows*_cols))
	{
		if(count == _cols)
			SetCursor(1, 0);

		SendChar(character++);
		HAL_Delay(delay);
		count++;
	  }
	  Clear();

	  return character;
}

/**
 * @brief Send a string value to the LCD screen
 * @param String	String value to send to the LCD
 */
void LCD_4bitComm::SendString(char *String)
{
	while(*String)
	{
		SendChar(*String++);
	}

}

/**
 * @brief Send an integer value to the LCD screen.
 * @param Num	Integer to send to the LCD screen.
 * @param len	Expected number of digits of the Num variable.
 */
void LCD_4bitComm::SendInt(int Num, uint8_t len)
{
	  char StringNum[len];
	  sprintf(StringNum, LCD_INT_FORMAT, Num);

	  SendString(StringNum);

}

/**
 * @brief	Send a float value to the LCD screen.
 * @param Num	float value to send to the LCD screen.
 * @param len	Expected number of digits of the Num variable.
 */
void LCD_4bitComm::SendFloat(float Num, uint8_t len)
{
	  char StringNum[len];
	  snprintf(StringNum, len + 1,  LCD_FLOAT_FORMAT, Num);

	  SendString(StringNum);

}

/**
 * @brief Initialization function for the LCD screen. This function will:
 * - Set the LCD to 4-bit mode
 * - Hide the blinking cursor
 * - Set the LCD to 2 Row mode
 * - Turn on the display
 * - Instructs the LCD to shift the address value by 1, so after each character
 * 	 is written, the cursor will automatically right shift by 1 space on the LCD screen.
 */
void LCD_4bitComm::Init(void)
{

	  HAL_Delay(LCD_INIT_STEP0_DELAY); // The delay helps the display boot up before beginning to send data to it
	  // Set to 4-bit mode
	  // First 4 bits ignored
	  SendIns(LCD_4BIT_MODE_CMD);
	  HAL_Delay(LCD_INIT_STEP1_DELAY);
	  // Set to 4-bit mode again, LCD inits as 8-bit mode, this needs to be sent twice to take proper effect.
	  // First nibble (7:4) sets LCD to 4-bit mode
	  // Second nibble (3:0) sets LCD to 2 row display
	  SendIns((LCD_4BIT_MODE_CMD << NIBBLE_SIZE) + LCD_2ROW_MODE_CMD);
	  SendIns(LCD_TURN_ON_DISP_CMD); // Turn on disp
	  SendIns(LCD_INC_ADDR_BY_1_CMD); // Set mode to inc addr by 1
	  Clear(); // clear LCD
}
