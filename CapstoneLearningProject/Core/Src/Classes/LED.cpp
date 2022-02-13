/*
 * LED.cpp
 * This is a simple test to verify the
 * ability to create and interact with
 * cpp classes in stm32
 *  Created on: Feb 10, 2022
 *      Author: elimo
 */

#include "LED.h"

LED::LED(GPIO_TypeDef *port, uint16_t pin, uint16_t HAL_PIN_Ref)
{
	_port = port;
	_pin = pin;
	_HAL_pin = HAL_PIN_Ref;

}

void LED::Init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(_port, _HAL_pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = _HAL_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(_port, &GPIO_InitStruct);
}


void LED::On()
{
	HAL_GPIO_WritePin(_port, _HAL_pin, GPIO_PIN_SET);
}

void LED:: Off()
{
	HAL_GPIO_WritePin(_port, _HAL_pin, GPIO_PIN_RESET);
}

int LED::test()
{
	return 43;
}

LED::~LED() {
	// TODO Auto-generated destructor stub
}

