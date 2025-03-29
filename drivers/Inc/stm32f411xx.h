/*
 * stm32f411xx.h
 *
 *  Created on: Mar 23, 2025
 *      Author: Aswin Sreeraj
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

//========================== Processor Specific Details =========================================
// Refer: ARM Cortex M4 user guide

// ARM Cortex Mx Processor NVIC ISERx register addresses
#define NVIC_ISER0							((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1							((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2							((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3							((volatile uint32_t*) 0xE000E10C)


// ARM Cortex Mx Processor NVIC ICERx register addresses
#define NVIC_ICER0							((volatile uint32_t*) 0XE000E180)
#define NVIC_ICER1							((volatile uint32_t*) 0XE000E184)
#define NVIC_ICER2							((volatile uint32_t*) 0XE000E188)
#define NVIC_ICER3							((volatile uint32_t*) 0XE000E18C)

// ARM Cortex Mx Processor Priority Register Address Calculation
#define NVIC_PR_BASE_ADDR					((volatile uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED				4

//===============================================================================================


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

// == GPIO ==
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

// == RCC ==
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

// == EXTI ==
typedef struct {
	volatile uint32_t IMR; 					// Interrupt mask												Address offset: 0x00
	volatile uint32_t EMR; 					// Event mask													Address offset: 0x04
	volatile uint32_t RTSR; 				// Rising trigger selection										Address offset: 0x08
	volatile uint32_t FTSR; 				// Falling trigger selection									Address offset: 0x0C
	volatile uint32_t SWIER; 				// Software interrupt event										Address offset: 0x10
	volatile uint32_t PR; 					// Pending (tigger request)										Address offset: 0x14
} EXTI_RegDef_t;

// == SYSCFG ==
typedef struct {
	volatile uint32_t MEMRMP;				// Memory Remap													Address Offset: 0x00;
	volatile uint32_t PMC;					// Pperipheral mode configuration								Address Offset: 0x04;
	volatile uint32_t EXTICR[4];			// External interrupt configuration								Address Offset: 0x08-0x14;
	uint32_t RESERVED[2];					// Address offset: 0x18, 0x1C
	volatile uint32_t CMPCR;				// Compensation cell control									Address Offset: 0x20;
} SYSCFG_RegDef_t;

// Peripheral Definitions

#define GPIOA 								((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 								((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 								((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 								((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 								((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH 								((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC									((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI								((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG								((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

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


#define GPIO_BASEADDR_TO_CODE(x)			( (x == GPIOA) ? 0 :\
											  (x == GPIOB) ? 1 :\
											  (x == GPIOC) ? 2 :\
											  (x == GPIOD) ? 3 :\
											  (x == GPIOE) ? 4 :\
											  (x == GPIOH) ? 7 :0 )

// Interrupt Request Number(IRQ) (from vector table)
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

// Interrupt priority levels
#define NVIC_IRQ_PR_0			0
#define NVIC_IRQ_PR_1			1
#define NVIC_IRQ_PR_2			2
#define NVIC_IRQ_PR_3			3
#define NVIC_IRQ_PR_4			4
#define NVIC_IRQ_PR_5			5
#define NVIC_IRQ_PR_6			6
#define NVIC_IRQ_PR_7			7
#define NVIC_IRQ_PR_8			8
#define NVIC_IRQ_PR_9			9
#define NVIC_IRQ_PR_10			10
#define NVIC_IRQ_PR_11			11
#define NVIC_IRQ_PR_12			12
#define NVIC_IRQ_PR_13			13
#define NVIC_IRQ_PR_14			14
#define NVIC_IRQ_PR_15			15


// Generic Macros
#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET 						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

#include "stm32f411xx_gpio_driver.h"

#endif /* INC_STM32F411XX_H_ */
