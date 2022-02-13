/*
 * BMP085.h
 *
 *  Created on: Feb 13, 2022
 *      Author: elimo
 */

#ifndef SRC_BMP085_H_
#define SRC_BMP085_H_

#define BMP085_DEFAULT_ADDRESS 0x77
#define BMP085_8BIT_ADDRES BMP085_DEFAULT_ADDRESS << 1

#define TEMP_READ_WAIT_TIME_MS 10 // Datasheet states to wait at least 4.5 ms

class BMP085 {
public:
	BMP085();
	void getCalData();
	float readTemperature();
};

#endif /* SRC_BMP085_H_ */
