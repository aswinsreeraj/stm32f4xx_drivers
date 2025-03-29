/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Mar 23, 2025
 *      Author: Aswin Sreeraj
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"


// Configuration structure for GPIO pin

typedef struct {
	uint8_t GPIO_PinNumber;						// Possible values from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;						// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;						// Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;				// Possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;						// Possible values from @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

// Handle Structure for a GPIO pin

typedef struct {
	GPIO_RegDef_t *pGPIOx;						// Holds the base address of the GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig;			// Hold the GPIO pin configuration settings
} GPIO_Handle_t;


// GPIO pin numbers
// @GPIO_PIN_NUMBER
#define GPIO_PIN_N0_0			0
#define GPIO_PIN_N0_1			1
#define GPIO_PIN_N0_2			2
#define GPIO_PIN_N0_3			3
#define GPIO_PIN_N0_4			4
#define GPIO_PIN_N0_5			5
#define GPIO_PIN_N0_6			6
#define GPIO_PIN_N0_7			7
#define GPIO_PIN_N0_8			8
#define GPIO_PIN_N0_9			9
#define GPIO_PIN_N0_10			10
#define GPIO_PIN_N0_11			11
#define GPIO_PIN_N0_12			12
#define GPIO_PIN_N0_13			13
#define GPIO_PIN_N0_14			14
#define GPIO_PIN_N0_15			15

// GPIO pin modes
// @GPIO_PIN_MODES
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

// GPIO output types
// @GPIO_PIN_OP_TYPE
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

// GPIO output speeds
// @GPIO_PIN_SPEED
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

// GPIO pull-up pull-down configuration
// @GPIO_PIN_PUPD
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


//==============================================================================================
//										API Prototypes
//==============================================================================================

// Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
