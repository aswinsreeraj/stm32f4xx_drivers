/*
 * led_button.c
 *
 *  Created on: Mar 28, 2025
 *      Author: Aswin Sreeraj
 */

#include "stm32f411xx.h"

#define BTN_PRESSED 0

void delay(void) {
	for(uint32_t i = 0; i < 500000/2; i++);
} // eo delay::

int main(void) {

	GPIO_Handle_t GpioLed, GpioBtn;

	// GPIO LED configuration
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; 		// GPIO_OP_TYPE_OD (open drain), connect the pin to V_CC via a 470Ohms resistor
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;		// Pull-up resistor is 40kOhms, won't activate LED

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);

	// GPIO button configuration
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1) {
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_N0_0) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_N0_13);
		} // eo if
	} // eo while

	return 0;
}// eo main::
