/*
 * BMP085.cpp
 *
 *  Created on: Feb 13, 2022
 *      Author: elimo
 */

#include "BMP085.h"

BMP085::BMP085(I2C_HandleTypeDef* hi2c)
{
	_hi2c = hi2c;

}

/**
 * @brief Most sensor readings in the MPU6050 are 16-bit. This function can be used to take in
 * data from two 8-bit registers and combine it into 1 16-bit value
 * @param high_reg	register holding the more significant 8 bits of the 16-bit reading
 * @param low_reg	register holding the less significant 8 bits of the 16-bit reading
 * @return	derived 16-bit value from combining the data in the two registers
 */
uint16_t BMP085::Read16BitValue(uint8_t high_reg, uint8_t low_reg)
{
	// Tell MPU6050 that we want to read from the temperature register
	_buf[0] = high_reg;
	_buf[1] = low_reg;
	_ret = HAL_I2C_Master_Transmit(_hi2c, BMP085_8BIT_ADDRES, _buf, 1, BMP085_MAX_TIMEOUT);
	if ( _ret != HAL_OK )
	{
	  strcpy((char*)_buf, "Error Tx\r\n");
	  _val = BMP085_ERROR_CODE;
	}
	else
	{
		// Read 2 bytes from the temperature register
		_ret = HAL_I2C_Master_Receive(_hi2c, BMP085_8BIT_ADDRES, _buf, 2, BMP085_MAX_TIMEOUT);
		if (_ret != HAL_OK )
		{
			strcpy((char*)_buf, "Error Rx\r\n");
			_val = BMP085_ERROR_CODE;
		}

		else
		{
			// Retrieve temp data from buffer
			_val = ((uint16_t)_buf[0] << BMP085_HIGH_REGISTER_OFFSET | _buf[1]);
		}
	}

		return _val;
}


void BMP085::getCalData()
{
	// collect all calibration data stored on BMP085 EEPROM
	 _ac1 = Read16BitValue(BMP085_RA_AC1_H,	BMP085_RA_AC1_L);
	 _ac2 = Read16BitValue(BMP085_RA_AC2_H, BMP085_RA_AC2_L);
	 _ac3 = Read16BitValue(BMP085_RA_AC3_H, BMP085_RA_AC3_L);
	 _ac4 = Read16BitValue(BMP085_RA_AC4_H, BMP085_RA_AC4_L);
	 _ac5 = Read16BitValue(BMP085_RA_AC5_H, BMP085_RA_AC5_L);
	 _ac6 = Read16BitValue(BMP085_RA_AC6_H, BMP085_RA_AC6_L);
	 _b1 = Read16BitValue(BMP085_RA_B1_H, BMP085_RA_B1_L);
	 _b2 = Read16BitValue(BMP085_RA_B2_H, BMP085_RA_B2_L);
	 _mb = Read16BitValue(BMP085_RA_MB_H, BMP085_RA_MB_L);
	 _mc = Read16BitValue(BMP085_RA_MC_H, BMP085_RA_MC_L);
	 _md = Read16BitValue(BMP085_RA_MD_H, BMP085_RA_MD_L);
}

void BMP085::Init()
{

}

float BMP085::readTemperature()
{
	// Write 0x2E into reg 0xF4
	// Wait at least 4.5 ms
	// Read reg 0xF6 (MSB) and reg 0xF7 (LSB)
	// Combine registers
	// Calculate True Temperature
	// Convert to deg F
	// Return True Temp val
}

