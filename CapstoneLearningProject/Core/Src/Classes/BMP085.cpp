/*
 * BMP085.cpp
 *
 *  Created on: Feb 13, 2022
 *      Author: elimo
 */

#include "BMP085.h"

BMP085::BMP085()
{
	// TODO Auto-generated constructor stub

}

void BMP085::getCalData()
{
	//
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

