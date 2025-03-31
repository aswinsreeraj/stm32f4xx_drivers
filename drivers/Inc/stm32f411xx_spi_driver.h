/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Mar 30, 2025
 *      Author: Aswin Sreeraj
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

// Configuration structure for SPIx peripheral
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

// Handle structure for SPIx peripheral
typedef struct {
	SPI_RegDef_t *pSPIx;					// Base address of SPIx peripheral
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

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
