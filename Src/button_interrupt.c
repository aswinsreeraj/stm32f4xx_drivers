/*
 * led_button.c
 *
 *  Created on: Mar 28, 2025
 *      Author: Aswin Sreeraj
 */

#include "stm32f411xx.h"
#include <string.h>

#define BTN_PRESSED 0

void delay(void) {
	// Delay of ~200ms when system clock is 16MHz
	for(uint32_t i = 0; i < 500000/2; i++);
} // eo delay::

int main(void) {

	GPIO_Handle_t GpioLed, GpioBtn;

	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

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
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_N0_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	// IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PR_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);

	return 0;
}// eo main::

/* EXTI9_5_IRQHandler:==========================================================
Author: Aswin Sreeraj
Date: 29/03/2025
Description: ISR
Input: 	None
Return: None
===================================================================================*/
void EXTI15_10_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(GPIO_PIN_N0_12);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_N0_13);
} // eo EXTI9_5_IRQHandler::
