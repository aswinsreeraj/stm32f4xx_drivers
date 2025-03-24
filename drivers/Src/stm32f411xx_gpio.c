/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Mar 23, 2025
 *      Author: Aswin Sreeraj
 */

#include "stm32f411xx_gpio_driver.h"


// Init and De-init
/* GPIO_Init:=======================================================================
Author: Aswin Sreeraj
Date: 23/03/2025
Description: Initiliaze the GPIO port
Input: GPIO_Handle_t *pGPIOHandle, handle to the GPIO port
Return: None
===================================================================================*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0; // Temporary register
	// Configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	} else {
		// Interrupt mode

	} // eo if-else

	temp = 0;
	// Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	// Configure pull-up pull-down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	// Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	// Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN) {
		uint32_t temp1, temp2;
		// Configure the alt function registers
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= (0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
} // eo GPIO_Init::

/* GPIO_DeInit:=====================================================================
Author: Aswin Sreeraj
Date: 23/03/2025
Description: Deinitialize the GPIO ports
Input: GPIO_RegDef_t *pGPIOx,
Return: None
===================================================================================*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} // eo if-else if
} // eo GPIO_DeInit::

// Peripheral clock setup
/* GPIO_PeriClockControl:===========================================================
Author: Aswin Sreeraj
Date: 23/03/2025
Description: Enable or Disable peripheral clock for the given GPIO port
Input: 	GPIO_RegDef_t *pGPIOx, baseaddress of GPIO peripheral
		uint8_t EnorDi, ENABLE or DISABLE macros
Return: None
===================================================================================*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} // eo if-else if
	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} // eo if-else if
	} // eo if-else

} // eo GPIO_PeriClockControl:

// Data read and write
/* GPIO_ReadFromInputPin:===========================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

} // eo GPIO_ReadFromInputPin::

/* GPIO_ReadFromInputPort:===========================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

} // eo GPIO_ReadFromInputPort::

/* GPIO_WriteToOutputPin:============================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {

} // eo GPIO_WriteToOutputPin::

/* GPIO_WriteToOutputPort:===========================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value) {

} // eo GPIO_WriteToOutputPort::

/* GPIO_ToggleOutputPin:=============================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

} // eo GPIO_ToggleOutputPin::

// IRQ Configuration and ISR handling
/* GPIO_IRQConfig:==================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

} // eo GPIO_IRQConfig::

/* GPIO_IRQHandling:=================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_IRQHandling(uint8_t PinNumber) {

} // eo GPIO_IRQHandling::
