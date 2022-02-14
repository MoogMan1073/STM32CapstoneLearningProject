/*
 * MPU6050.cpp
 *
 *  Created on: Feb 12, 2022
 *      Author: elimo
 */

#include "MPU6050.h"

/**
 * @brief Constructor for the MPU6050 device. Note this constructor assumes
 * i2c communication has already been configured before the device is initialized.
 * @param hi2c	i2c line used to communicate with the MPU6050.
 */
MPU6050::MPU6050(I2C_HandleTypeDef* hi2c) {

	_hi2c = hi2c;

}

/**
 * @brief Initialization function for the MPU6050 device. The device must first be reset by
 * sending a 0x00 value to the power management 1 register. From there, any desired configuration
 * for the sensors on the MPU6050 can be applied.
 */
void MPU6050::Init()
{
	  // Initialize MPU6050
	  	_buf[0] =  MPU6050_RA_PWR_MGMT_1;
	  	_buf[1] = 0x00;
		_ret = HAL_I2C_Master_Transmit(_hi2c, MPU6050_8BIT_ADDRESS, _buf, 2, HAL_MAX_DELAY);
		if ( _ret != HAL_OK )
		{
		  strcpy((char*)_buf, "Error Tx\r\n");
		}
		else
		{
			uint8_t mpu6050_pwr_mgmt_1_config = (0 << MPU6050_PWR1_SLEEP_BIT) | MPU6050_CLOCK_PLL_XGYRO;

			_buf[0] = MPU6050_RA_ACCEL_CONFIG;
			_buf[1] = MPU6050_ACCEL_FS_2 << 3;
			_buf[2] = MPU6050_RA_GYRO_CONFIG;
			_buf[3] = MPU6050_GYRO_FS_500 << 3;
			_buf[4] = MPU6050_RA_PWR_MGMT_1;
			_buf[5] = mpu6050_pwr_mgmt_1_config;

			_ret = HAL_I2C_Master_Transmit(_hi2c, MPU6050_8BIT_ADDRESS, _buf, 6, HAL_MAX_DELAY);
			if ( _ret != HAL_OK )
			{
			  strcpy((char*)_buf, "Error Tx\r\n");
			}
		}
}

/**
 * @brief Convert a 16-bit value to its 2's complement
 * @param val	value to convert
 * @return		2's complement of val
 */
uint16_t MPU6050::Convert2sComplement(uint16_t val)
{
	if( val > MPU6050_2S_COMPLEMENT_THRESH)
		return val |= MPU6050_2S_COMPLEMENT_CONVERSION;
	else
		return val;
}

/**
 * @brief Most sensor readings in the MPU6050 are 16-bit. This function can be used to take in
 * data from two 8-bit registers and combine it into 1 16-bit value
 * @param high_reg	register holding the more significant 8 bits of the 16-bit reading
 * @param low_reg	register holding the less significant 8 bits of the 16-bit reading
 * @return	derived 16-bit value from combining the data in the two registers
 */
uint16_t MPU6050::Read16BitValue(uint8_t high_reg, uint8_t low_reg)
{
	// Tell MPU6050 that we want to read from the temperature register
	_buf[0] = high_reg;
	_buf[1] = low_reg;
	_ret = HAL_I2C_Master_Transmit(_hi2c, MPU6050_8BIT_ADDRESS, _buf, 1, MPU6050_MAX_TIMEOUT);
	if ( _ret != HAL_OK )
	{
	  strcpy((char*)_buf, "Error Tx\r\n");
	  _val = MPU6050_ERROR_CODE;
	}
	else
	{
		// Read 2 bytes from the temperature register
		_ret = HAL_I2C_Master_Receive(_hi2c, MPU6050_8BIT_ADDRESS, _buf, 2, MPU6050_MAX_TIMEOUT);
		if (_ret != HAL_OK )
		{
			strcpy((char*)_buf, "Error Rx\r\n");
			_val = MPU6050_ERROR_CODE;
		}

		else
		{
			// Retrieve temp data from buffer
			_val = ((uint16_t)_buf[0] << MPU6050_HIGH_REGISTER_OFFSET | _buf[1]);
		}
	}

		return _val;
}

/**
 * @brief Convert a raw 16-bit accelerometer reading into terms of g
 * @param raw_val	raw 16-bit value from sensor registers
 * @return	value from register converted to terms of g
 */
float MPU6050::getAccelinUnits(uint16_t raw_val)
{
	float accel_in_units = 0;

	if(raw_val != MPU6050_ERROR_CODE)
	{
		accel_in_units = ((float)raw_val) / MPU6050_ACCEL_AFS_2_SCALE;
	}
	return accel_in_units;
}

/**
 * @brief Convert a raw 16-bit gyroscope reading into terms of degrees per second
 * @param raw_val	raw 16-bit value from sensor registers
 * @return	value from register converted to terms of degrees per second
 */
float MPU6050::getGyroinUnits(uint16_t raw_val)
{
	float gyro_in_units = 0;

	if(raw_val != MPU6050_ERROR_CODE)
	{
		gyro_in_units = ((float)raw_val) / MPU6050_GYRO_FS_SEL_1_SCALE;
	}

	return gyro_in_units;
}

/**
 * @brief Accesses registers on MPU6050 relating to the temperature reading
 * @return Temperature value in terms of degrees Fahrenheit.
 */
float MPU6050::readTemperature()
{
	float temperature;

	// Tell MPU6050 that we want to read from the temperature register
	temperature = Read16BitValue(MPU6050_RA_TEMP_OUT_H, MPU6050_RA_TEMP_OUT_L);

	if(_val != MPU6050_ERROR_CODE)
	{
		// process data into temperature with readable units
		temperature = (temperature/MPU6050_TEMP_SCALE) - MPU6050_TEMP_OFFSET;
		temperature = (temperature * MPU6050_CEL_TO_FAR_CONVERSION);
	}

	return temperature;
}

/**
 * @brief Accesses registers on MPU6050 relating to the Accelerometer reading
 * in the X-direction
 * @return Accelerometer value in terms of g's.
 */
float MPU6050::readAccelerometerX()
{
	uint16_t accelXval;
	float accelXg = 0;

	// Tell MPU6050 that we want to read from the accel x register
	accelXval = Read16BitValue(MPU6050_RA_ACCEL_XOUT_H, MPU6050_RA_ACCEL_XOUT_L);
	accelXg = getAccelinUnits(accelXval);
	return accelXg;
}

/**
 * @brief Accesses registers on MPU6050 relating to the Accelerometer reading
 * in the Y-direction
 * @return Accelerometer value in terms of g's.
 */
float MPU6050::readAccelerometerY()
{
	uint16_t accelYval;
	float accelYg = 0;
	// Tell MPU6050 that we want to read from the accel y register
	accelYval = Read16BitValue(MPU6050_RA_ACCEL_YOUT_H, MPU6050_RA_ACCEL_YOUT_L);
	accelYg = getAccelinUnits(accelYval);
	return accelYg;
}

/**
 * @brief Accesses registers on MPU6050 relating to the Accelerometer reading
 * in the Z-direction
 * @return Accelerometer value in terms of g's.
 */
float MPU6050::readAccelerometerZ()
{
	uint16_t accelZval;
	float accelZg = 0;
	// Tell MPU6050 that we want to read from the accel z register
	accelZval = Read16BitValue(MPU6050_RA_ACCEL_ZOUT_H, MPU6050_RA_ACCEL_ZOUT_L);
	accelZg = getAccelinUnits(accelZval);
	return accelZg;
}

/**
 * @brief Accesses registers on MPU6050 relating to the gyroscope reading
 * in the X-direction
 * @return Accelerometer value in terms of degrees per second.
 */
float MPU6050::readGyroX()
{
	uint16_t gyroXval;
	float gyroXg = 0;

	// Tell MPU6050 that we want to read from the gyro x register
	gyroXval = Read16BitValue(MPU6050_RA_GYRO_XOUT_H, MPU6050_RA_GYRO_XOUT_L);
	gyroXg = getGyroinUnits(gyroXval);

	return gyroXg;
}

/**
 * @brief Accesses registers on MPU6050 relating to the gyroscope reading
 * in the Y-direction
 * @return Accelerometer value in terms of degrees per second.
 */
float MPU6050::readGyroY()
{
	uint16_t gyroYval;
	float gyroYg = 0;

	// Tell MPU6050 that we want to read from the gyro y register
	gyroYval = Read16BitValue(MPU6050_RA_GYRO_YOUT_H, MPU6050_RA_GYRO_YOUT_L);
	gyroYg = getGyroinUnits(gyroYval);

	return gyroYg;
}

/**
 * @brief Accesses registers on MPU6050 relating to the gyroscope reading
 * in the Z-direction
 * @return Accelerometer value in terms of degrees per second.
 */
float MPU6050::readGyroZ()
{
	uint16_t gyroZval;
	float gyroZg = 0;

	// Tell MPU6050 that we want to read from the gyro z register
	gyroZval = Read16BitValue(MPU6050_RA_GYRO_ZOUT_H, MPU6050_RA_GYRO_ZOUT_L);
	gyroZg = getGyroinUnits(gyroZval);

	return gyroZg;
}
