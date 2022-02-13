/*
 * LED.h
 *
 *  Created on: Feb 10, 2022
 *      Author: elimo
 */

#ifndef SRC_LED_H_
#define SRC_LED_H_

#include "stm32f030x8.h"
#include "stm32f0xx_hal.h"

class LED {
public:
	LED(GPIO_TypeDef *port, uint16_t pin, uint16_t HAL_PIN_Ref);
	void Init();
	void On();
	void Off();
	int test();
	virtual ~LED();
private:
	GPIO_TypeDef* _port;
	uint16_t _pin;
	uint16_t _HAL_pin;
};

#endif /* SRC_LED_H_ */
