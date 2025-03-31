/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Mar 30, 2025
 *      Author: Aswin Sreeraj
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

// Configuration structure for SPIx peripheral
typedef struct {
	uint8_t SPI_DeviceMode;					// Refer: @SPI_DeviceMode
	uint8_t SPI_BusConfig;					// Refer: @SPI_BusConfig
	uint8_t SPI_SclkSpeed;					// Refer: @SPI_SclkSpeed
	uint8_t SPI_DFF;						// Refer: @SPI_DFF			Data Frame Format
	uint8_t SPI_CPOL;						// Refer: @SPI_CPOL			Clock Polarity
	uint8_t SPI_CPHA;						// Refer: @SPI_CPHA			Clock Phase
	uint8_t SPI_SSM;						// Refer: @SPI_SSM			Slave Selection Management
} SPI_Config_t;

// Handle structure for SPIx peripheral
typedef struct {
	SPI_RegDef_t *pSPIx;					// Base address of SPIx peripheral
	SPI_Config_t SPIConfig;
} SPI_Handle_t;


// @SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

// @SPI_BusConfig
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		4

// @SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

// @SPI_DFF
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

// @SPI_CPOL
#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

// @SPI_CPHA
#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1

// @SPI_SSM
#define SPI_SSM_DI							0
#define SPI_SSM_EN							1

//==============================================================================================
//										API Prototypes
//==============================================================================================

// Init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Data send and receive
// Blocking type(non-interrupt based)
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

// Other peripheral control APIS


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
