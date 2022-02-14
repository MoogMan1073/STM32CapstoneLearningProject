/*
 * BMP085.cpp
 *
 *  Created on: Feb 13, 2022
 *      Author: elimo
 */

#include "BMP085.h"

/**
 * @brief	Constructor for the BMP085 temperautre & pressure sensor.
 * Note this constructor assumes i2c communication has already been configured
 * before the device is initialized.
 * @param hi2c	i2c line used to communicate with the BMP085.
 */
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
	_ret = HAL_I2C_Master_Transmit(_hi2c, BMP085_8BIT_ADDRESS, _buf, 1, BMP085_MAX_TIMEOUT);
	if ( _ret != HAL_OK )
	{
	  strcpy((char*)_buf, "Error Tx\r\n");
	  _val = BMP085_ERROR_CODE;
	}
	else
	{
		// Read 2 bytes from the temperature register
		_ret = HAL_I2C_Master_Receive(_hi2c, BMP085_8BIT_ADDRESS, _buf, 2, BMP085_MAX_TIMEOUT);
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

/**
 * @brief Before reading any sensor data. It is required to pull calibration data
 * from various registers stored on an EEPROM on the BMP085 device. This function
 * assigns all EEPROM calibration data to private variables in the BMP085 class.
 */
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

/**
 * @brief Initialization function for the BMP085 device. The only thing we do here
 * is pull in all of the calibration data stored on the BMP085.
 */
void BMP085::Init()
{
	getCalData();
}

/**
 * @brief Read the temperature data from the BMP085 and then convert it into
 * degree Fahrenheit.
 * @return Temperature read from the BMP085 in degress Fahrenheit.
 */
float BMP085::readTemperature()
{
	float temperature = 0;
	uint16_t raw_temp;
	// Taking var names directly from data sheet
	uint16_t X1;
	uint16_t X2;
	uint16_t T;

	// Write 0x2E into reg 0xF4
	_buf[0] = BMP085_READ_TEMP_CMD;
	_buf[1] = BMP085_RA_REQ_DATA;

	_ret = HAL_I2C_Master_Transmit(_hi2c, BMP085_8BIT_ADDRESS, _buf, 2, HAL_MAX_DELAY);
	if ( _ret != HAL_OK )
	{
	  strcpy((char*)_buf, "Error Tx\r\n");
	  return BMP085_ERROR_CODE;
	}

	// Wait at least 4.5 ms
	HAL_Delay(BMP085_TEMP_READ_WAIT_TIME_MS);
	// Read reg 0xF6 (MSB) and reg 0xF7 (LSB) + Combine registers
	raw_temp = Read16BitValue(BMP085_RA_DATA_H, BMP085_RA_DATA_L);

	// Calculate True Temperature, method taken from datasheet.
	X1 = (raw_temp - _ac6) * (_ac5 / (1<<15));
	X2 = (_mc * (1 << 11)) / (X1 + _md);
	_b5 = X1 + X2;
	T = (_b5 + 8) / (1 << 4); // yields temperature in increments of 0.1 deg C.
	temperature = ((float)T) / 10.0;
	// Convert to deg F
	temperature = (temperature * BMP085_CEL_TO_FAR_CONVERSION);

	// Return True Temp val
	return temperature;
}

/**
 * @brief Read the pressure data from the BMP085 and then convert it into hPa.
 * @return Pressure read from the BMP085 in hPa
 */
float BMP085::readPressure()
{
	float pressure = 0;
	int pressure_Pa;
	uint16_t raw_pressure;
	uint16_t B3;
	uint16_t B4;
	uint16_t B6;
	uint16_t B7;
	uint16_t X1;
	uint16_t X2;
	uint16_t X3 = 0;

	// Write 0x34 into reg 0xF4
	_buf[0] = BMP085_READ_TEMP_CMD;
	_buf[1] = BMP085_RA_REQ_DATA;

	_ret = HAL_I2C_Master_Transmit(_hi2c, BMP085_8BIT_ADDRESS, _buf, 2, HAL_MAX_DELAY);
	if ( _ret != HAL_OK )
	{
	  strcpy((char*)_buf, "Error Tx\r\n");
	  return BMP085_ERROR_CODE;
	}

	// wait at least 4.5 ms (only when not using any oversampling)
	HAL_Delay(BMP085_TEMP_READ_WAIT_TIME_MS);

	// read 0xF6 and 0xF7
	// NOTE: if we were to use oversampling, 0xF8 would also need to be considered.
	// I'm not too worried about getting more fidelity, a 16-bit value for pressure
	// will do just fine.
	// Check BMP085 datasheet pg. 13 for oversampling implementation if ever needed.
	raw_pressure = Read16BitValue(BMP085_RA_DATA_H, BMP085_RA_DATA_L);

	// Calculation taken from datasheet
	B6 = _b5 - 4000;
	X1 = (_b2 * (B6 * B6/(1 << 12))) / (1<< 11);
	X2 = _ac2 * B6/(1 << 11);
	X3 = X1 + X3;
	B3 = ((_ac1*4+X3) << 2) / 4;
	X1 = _ac3 * B6 / (1 << 13);
	X2 = (_b1 * (B6 * B6/(1 << 12))) / (1 << 16);
	X3 = ((X1 + X2) + 2) / (1 << 2);
	B4 = _ac4 * (unsigned long)(X3 + 32768) / (1 << 15);
	B7 = ((unsigned long) raw_pressure - B3) * (50000);
	if(B7 < 0x80000000)
	{
		pressure_Pa = (B7 * 2) / B4;
	}
	else
	{
		pressure_Pa = (B7/B4) * 2;
	}
	X1 = (pressure_Pa / (1 << 8)) * (pressure_Pa / (1 << 8));
	X1 = (X1 * 3038) / (1 << 16);
	X2 = (-7357 * pressure_Pa) / (1 << 16);
	pressure_Pa = pressure_Pa + (X1 + X2 + 3791) / (1 << 4);

	pressure = pressure_Pa * 0.01;

	return pressure;
}

