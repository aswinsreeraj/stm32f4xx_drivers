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
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// 1. Configure the FT selection register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// 1. // 1. Configure the RT selection register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// 1. Configure both FTSR & RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} // eo if-else-if

		// 2. COnfigure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

		// 3. Enable the EXTI interrupt delivery using Interrupt Mask Rsgister
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

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
Input: GPIO_RegDef_t *pGPIOx, base address of GPIO peripheral
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
Input: 	GPIO_RegDef_t *pGPIOx, base address of GPIO peripheral
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
Date: 23/03/2025
Description: Read the data from individual pin
Input: 	GPIO_RegDef_t *pGPIOx, base address of GPIO peripheral
		uint8_t PinNumber, pin number to be read from
Return: uint8_t, return the pin value
===================================================================================*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
} // eo GPIO_ReadFromInputPin::

/* GPIO_ReadFromInputPort:===========================================================
Author: Aswin Sreeraj
Date: 23/03/2025
Description: Read data from the entire port
Input: GPIO_RegDef_t *pGPIOx, base address of the GPIO peripheral
Return: uint16_t, return the port value
===================================================================================*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
} // eo GPIO_ReadFromInputPort::

/* GPIO_WriteToOutputPin:============================================================
Author: Aswin Sreeraj
Date: 23/03/2025
Description: Write value to the individual pin of GPIO port
Input: 	GPIO_RegDef_t *pGPIOx, base address of GPIO peripheral
		uint8_t PinNumber, pin number to written to
		uint8_t Value, value to be written (1 0r 0)
Return: None
===================================================================================*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if(Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	} // if-else
} // eo GPIO_WriteToOutputPin::

/* GPIO_WriteToOutputPort:===========================================================
Author: Aswin Sreeraj
Date: 23/03/2025
Description: Write value to the GPIO port
Input: 	GPIO_RegDef_t *pGPIOx, base address of GPIO peripheral
		uint8_t Value, value to be written to the port
Return: None
===================================================================================*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value) {
	pGPIOx->ODR = Value;
} // eo GPIO_WriteToOutputPort::

/* GPIO_ToggleOutputPin:=============================================================
Author: Aswin Sreeraj
Date: 23/03/2025
Description: Toggle the value of the individual pin
Input:	GPIO_RegDef_t *pGPIOx, base address of GPIO peripheral
		uint8_t PinNumber, pin to be toggled
Return: None
===================================================================================*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
} // eo GPIO_ToggleOutputPin::

// IRQ Configuration and ISR handling
/* GPIO_IRQInterruptConfig:==========================================================
Author: Aswin Sreeraj
Date: 29/03/2025
Description: Configure the interrupt
Input: 	uint8_t IRQNumber, IRQ for the specific EXTI
		uint8_t EnorDi, Enable or disable the interrupt
Return: None
===================================================================================*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(IRQNumber <= 31) {
			// Program NVIC_ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64) {
			// Program NVIC_ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber >= 64 && IRQNumber <= 96) {
			// Program NVIC_ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		} // eo if-else-if
	} else {
		if(IRQNumber <= 31) {
			// Program NVIC_ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64) {
			// Program NVIC_ICER1
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		} else if(IRQNumber >= 64 && IRQNumber <= 96) {
			// Program NVIC_ICER2
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		} // eo if-else-if
	} // eo if-else
} // eo GPIO_IRQInterruptConfig::

/* GPIO_IRQPriorityConfig:==========================================================
Author: Aswin Sreeraj
Date: 29/03/2025
Description: Configure the interrupt priority
Input: 	uint8_t IRQNumber,  IRQ for the specific EXTI
		uint8_t IRQPriority, for setting the priority of the interrupt
Return: None
===================================================================================*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// Calculate IPR register number
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
} // eo GPIO_IRQPriorityConfig::

/* GPIO_IRQHandling:=================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_IRQHandling(uint8_t PinNumber) {

	// Clear the EXTI pending register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	} // eo if
} // eo GPIO_IRQHandling::
