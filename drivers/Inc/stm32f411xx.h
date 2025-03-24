/*
 * stm32f411xx.h
 *
 *  Created on: Mar 23, 2025
 *      Author: Aswin Sreeraj
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

// Base addresses of flash and SRAM memories


#define FLASH_BASEADDR						0x08000000U  // Page #42 (RM)
#define SRAM1_BASEADDR						0x20000000U  // Page #42 (RM)
#define ROM_BASEADDR						0x1FFF0000U  // Page #42 (RM)
#define SRAM 								SRAM1_BASEADDR

// AHBx and PBx Bus Peripheral base addresses


#define PERIPH_BASEADDR							0x40000000U  // Page #39 (RM)
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR  // Page #39 (RM)
#define APB2PERIPH_BASEADDR						0x40010000U  // Page #39 (RM)
#define AHB1PERIPH_BASEADDR						0x40020000U  // Page #38 (RM)
#define AHB2PERIPH_BASEADDR						0x50000000U  // Page #38 (RM)


// Base addresses of peripherals on AHB1 bus

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)  // Page #38 (RM)
#define	GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)  // Page #38 (RM)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)  // Page #38 (RM)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)  // Page #38 (RM)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)  // Page #38 (RM)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)  // Page #38 (RM)

#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)  // Page #38 (RM)


// Base addresses of peripherals on APB1 bus

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)  // Page #39 (RM)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)  // Page #39 (RM)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)  // Page #39 (RM)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)  // Page #39 (RM)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)  // Page #39 (RM)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)  // Page #39 (RM)



// Base addresses of peripherals on APB2 bus

#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)  // Page #39 (RM)

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)  // Page #39 (RM)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)  // Page #39 (RM)
#define SPI5_BASEADDR						(APB2PERIPH_BASEADDR + 0x5000)  // Page #39 (RM)

#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)  // Page #39 (RM)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)  // Page #39 (RM)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)  // Page #39 (RM)


//======================= Peripheral Register Definition Structures ===========================

typedef struct {
	volatile uint32_t MODER;				// GPIO port mode register						Address offset: 0x00
	volatile uint32_t OTYPER;				// GPIO port output type register				Address offset: 0x04
	volatile uint32_t OSPEEDR;				// GPIO port output speed register				Address offset: 0x08
	volatile uint32_t PUPDR;				// GPIO port pull-up/pull-down register			Address offset: 0x0C
	volatile uint32_t IDR;					// GPIO port input data register				Address offset: 0x10
	volatile uint32_t ODR;					// GPIO port output data register				Address offset: 0x14
	volatile uint32_t BSRR;					// GPIO port bit set/reset register				Address offset: 0x18
	volatile uint32_t LCKR;					// GPIO port configuration lock register		Address offset: 0x1C
	volatile uint32_t AFR[2];				// GPIO alternate function register(low,high)	Address offset: 0x20, 0x24
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR;					// RCC clock control register									Address offset: 0x00
	volatile uint32_t PLLCFGR;				// RCC PLL configuration register								Address offset: 0x04
	volatile uint32_t CFGR;					// RCC clock configuration register								Address offset: 0x08
	volatile uint32_t CIR;					// RCC clock interrupt register									Address offset: 0x0C
	volatile uint32_t AHB1RSTR;				// RCC AHB1 peripheral reset register							Address offset: 0x10
	volatile uint32_t AHB2RSTR;				// RCC AHB2 peripheral reset register							Address offset: 0x14
	uint32_t RESERVED0[2];
	volatile uint32_t APB1RSTR;				// RCC APB1 peripheral reset register							Address offset: 0x20
	volatile uint32_t APB2RSTR;				// RCC APB2 peripheral reset register							Address offset: 0x24
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;				// RCC AHB1 peripheral clock enable register					Address offset: 0x30
	volatile uint32_t AHB2ENR;				// RCC AHB2 peripheral clock enable register					Address offset: 0x34
	uint32_t RESERVED2[2];
	volatile uint32_t APB1ENR;				// RCC APB1 peripheral clock enable register					Address offset: 0x40
	volatile uint32_t APB2ENR;				// RCC APB2 peripheral clock enable register					Address offset: 0x44
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;			// RCC AHB1 peripheral clock enable in low power mode register	Address offset: 0x50
	volatile uint32_t AHB2LPENR;			// RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54
	uint32_t RESERVED4[2];
	volatile uint32_t APB1LPENR;			// RCC APB1 peripheral clock enable in low power mode register	Address offset: 0x60
	volatile uint32_t APB2LPENR;			// RCC APB2 peripheral clock enable in low power mode register	Address offset: 0x64
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;					// RCC Backup domain control register							Address offset: 0x70
	volatile uint32_t CSR;					// RCC clock control & status register							Address offset: 0x74
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;				// RCC spread spectrum clock generation register				Address offset: 0x80
	volatile uint32_t PLLI2SCFGR;			// RCC PLLI2S configuration register							Address offset: 0x84
} RCC_RegDef_t;

// Peripheral Definitions

#define GPIOA 								((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 								((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 								((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 								((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 								((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH 								((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC									((RCC_RegDef_t*) RCC_BASEADDR)

// Clock enable macros for GPIOx peripherals

#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |= (1 << 7))

// Clock enable macros for I2Cx peripherals

#define I2C1_PCLK_EN()						(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()						(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()						(RCC->APB1ENR |= (1 << 23))

// Clock enable macros for SPIx peripherals

#define SPI1_PCLK_EN()						(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()						(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()						(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()						(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()						(RCC->APB2ENR |= (1 << 20))

// Clock enable macros for USARTx peripherals

#define USART1_PCLK_EN()					(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()					(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()					(RCC->APB2ENR |= (1 << 5))

// Clock enable macros for SYSCFG peripherals

#define SYSCFG_PCLK_EN()					(RCC->APB2ENR |= (1 << 14))

// Clock disable macros for GPIOx peripherals

#define GPIOA_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 7))

// Clock disable macros for I2Cx peripherals

#define I2C1_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 23))

// Clock disable macros for SPIx peripherals

#define SPI2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 20))

// Clock disable macros for SYSCFG peripherals

#define SYSCFG_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 14))

// Macros for resetting GPIOx peripherals
#define GPIOA_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()					do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

#endif /* INC_STM32F411XX_H_ */
