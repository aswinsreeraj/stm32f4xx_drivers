/*
 * led_toggle.c
 *
 *  Created on: Mar 24, 2025
 *      Author: Aswin Sreeraj
 */

#include "stm32f411xx.h"

void delay(void) {
	for(uint32_t i = 0; i < 500000; i++);
} // eo delay::

int main(void) {

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; 		// GPIO_OP_TYPE_OD (open drain), connect the pin to V_CC via a 470Ohms resistor
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;		// Pull-up resistor is 40kOhms, won't activate LED

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);

	while(1) {
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_N0_13);
		delay();
	} // eo while

	return 0;
}// eo main::
