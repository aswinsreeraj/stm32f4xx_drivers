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
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

} // eo GPIO_Init::

/* GPIO_DeInit:=====================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

} // eo GPIO_DeInit::

// Peripheral clock setup
/* GPIO_PeriClockControl:===========================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {

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

/* GPIO_WriteToOutputPort:==================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value) {

} // eo GPIO_WriteToOutputPort::

/* GPIO_ToggleOutputPin:==================================================================
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

/* GPIO_IRQHandling:==================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void GPIO_IRQHandling(uint8_t PinNumber) {

} // eo GPIO_IRQHandling::
