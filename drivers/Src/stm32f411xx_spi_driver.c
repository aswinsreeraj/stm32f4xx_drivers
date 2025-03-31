/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Mar 30, 2025
 *      Author: Aswin Sreeraj
 */

#include "stm32f411xx_spi_driver.h"


// Init and De-init
/* SPI_Init:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_Init(SPI_Handle_t *pSPIHandle) {

} // eo SPI_Init::

/* SPI_DeInit:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

} // eo SPI_DeInit:

// Peripheral clock setup
/* SPI_PeriClockControl:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if(pSPIx == SPI4) {
			SPI4_PCLK_EN();
		} else if(pSPIx == SPI5) {
			SPI5_PCLK_EN();
		} // eo if-else if
	} else {
		if(pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if(pSPIx == SPI4) {
			SPI4_PCLK_DI();
		} else if(pSPIx == SPI5) {
			SPI5_PCLK_DI();
		} // eo if-else if
	} // eo if-else

} // eo SPI_PeriClockControl::

// Data send and receive
/* SPI_SendData:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

} // eo SPI_SendData::

/* SPI_ReceiveData:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

} // eo SPI_ReceiveData::

// IRQ configuration and ISR handling
/* SPI_IRQInterruptConfig:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

} // eo SPI_IRQInterruptConfig::

/* SPI_IRQPriorityConfig:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

} // eo SPI_IRQPriorityConfig

/* SPI_IRQHandling:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

} // eo SPI_IRQHandling::
