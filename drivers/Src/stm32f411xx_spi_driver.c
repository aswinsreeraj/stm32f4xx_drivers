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
Date: 30/03/2025
Description: Initialize the SPI peripheral
Input: SPI_Handle_t *pSPIHandle, handle for the SPI peripheral
Return: None
===================================================================================*/
void SPI_Init(SPI_Handle_t *pSPIHandle) {

	// Configure SPI_CR1 register
	uint32_t tempreg = 0;

	// Configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// Configure the bus
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// Clear BIDI mode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// Set the BIDI mode
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// Clear the BIDI mode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// Set RXONLY bit
		tempreg |= (1 << SPI_CR1_RXONLY);
	} // eo if-else-if

	// Configure the serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SPI_SclkSpeed << SPI_CR1_BR;

	// Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// Configure the clock polarity(idle state)
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// Configure the clock phase (trailing edge or leading edge)
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

} // eo SPI_Init::

/* SPI_DeInit:=======================================================================
Author: Aswin Sreeraj
Date:
Description:
Input:
Return: None
===================================================================================*/
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

	if(pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if(pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if(pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if(pSPIx == SPI4) {
		SPI4_REG_RESET();
	} else if(pSPIx == SPI5) {
		SPI5_REG_RESET();
	} // eo if-else if

} // eo SPI_DeInit:

// Peripheral clock setup
/* SPI_PeriClockControl:=======================================================================
Author: Aswin Sreeraj
Date: 30/03/2025
Description: Configure the clock for the SPI peripheral
Input:	SPI_RegDef_t *pSPIx, base address of the SPI peripheral
		uint8_t EnorDi, to enable or disable the peripheral clock
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
