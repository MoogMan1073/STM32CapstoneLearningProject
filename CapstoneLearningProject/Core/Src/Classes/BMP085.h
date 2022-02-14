/*
 * BMP085.h
 *
 *  Created on: Feb 13, 2022
 *      Author: elimo
 */

#ifndef SRC_BMP085_H_

#include "string.h"

#include "stm32f030x8.h"
#include "stm32f0xx_hal.h"

#define SRC_BMP085_H_

#define BMP085_DEFAULT_ADDRESS 0x77
#define BMP085_8BIT_ADDRESS BMP085_DEFAULT_ADDRESS << 1
#define BMP085_RA_AC1_H 0xAA
#define BMP085_RA_AC1_L 0xAB
#define BMP085_RA_AC2_H 0xAC
#define BMP085_RA_AC2_L 0xAD
#define BMP085_RA_AC3_H 0xAE
#define BMP085_RA_AC3_L 0xAF
#define BMP085_RA_AC4_H 0xB0
#define BMP085_RA_AC4_L 0xB1
#define BMP085_RA_AC5_H 0xB2
#define BMP085_RA_AC5_L 0xB3
#define BMP085_RA_AC6_H 0xB4
#define BMP085_RA_AC6_L 0xB5
#define BMP085_RA_B1_H 0xB6
#define BMP085_RA_B1_L 0xB7
#define BMP085_RA_B2_H 0xB8
#define BMP085_RA_B2_L 0xB9
#define BMP085_RA_MB_H 0xBA
#define BMP085_RA_MB_L 0xBB
#define BMP085_RA_MC_H 0xBC
#define BMP085_RA_MC_L 0xBD
#define BMP085_RA_MD_H 0xBE
#define BMP085_RA_MD_L 0xBF
#define BMP085_RA_REQ_DATA 0xF4 // use for both temperature and pressure readings
#define BMP085_RA_DATA_H 0xF6 // Returns either temperature or pressure data based on whats
#define BMP085_RA_DATA_L 0xF7	// Written to the Request Data register


#define BMP085_READ_TEMP_CMD 0x2E
#define BMP085_READ_PRESS_CMD 0x34

#define BMP085_CEL_TO_FAR_CONVERSION (9.0/5.0) + 32.0

#define BMP085_BUFFER_SIZE 32
#define BMP085_HIGH_REGISTER_OFFSET 8
#define BMP085_MAX_TIMEOUT 5000
#define BMP085_ERROR_CODE 0xFFFF

#define BMP085_TEMP_READ_WAIT_TIME_MS 10 // Datasheet states to wait at least 4.5 ms

class BMP085 {
public:
	BMP085(I2C_HandleTypeDef* hi2c);
	uint16_t Read16BitValue(uint8_t high_reg, uint8_t low_reg);
	void getCalData();
	void Init();
	float readTemperature();
	float readPressure();

private:
	uint8_t _buf[BMP085_BUFFER_SIZE] = {};
	uint16_t _val;
	I2C_HandleTypeDef* _hi2c;
	HAL_StatusTypeDef _ret;

	// Calibration Data stored in EEPROM
	uint16_t _ac1;
	uint16_t _ac2;
	uint16_t _ac3;
	uint16_t _ac4;
	uint16_t _ac5;
	uint16_t _ac6;
	uint16_t _b1;
	uint16_t _b2;
	uint16_t _b5;
	uint16_t _mb;
	uint16_t _mc;
	uint16_t _md;
};

#endif /* SRC_BMP085_H_ */
