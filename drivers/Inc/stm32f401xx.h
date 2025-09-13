/*
 * stm32f401xx.h
 *
 *  Created on: Jun 10, 2024
 *      Author: mohitpassan
 */

#include <stdint.h>
#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#define __vo volatile

// Main memories
#define FLASH_BASEADDR			0x08000000U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM					SRAM1_BASEADDR

// Peripherals addresses
#define PERIPH_BASEADDR			0x40000000U
#define APB1_PERIPH_BASEADDR	PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR	0x40010000U
#define AHB1_PERIPH_BASEADDR	0x40020000U
#define AHB2_PERIPH_BASEADDR	0x50000000U

// AHB1 peripheral addresses
#define GPIOA_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x3800)

// APB1 peripheral addresses
#define I2C1_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x4400)

// APB2 peripheral addresses
#define EXTI_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3800)
#define SPI1_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR			(APB1_PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB1_PERIPH_BASEADDR + 0x1400)

// GPIO peripheral register definition structure
typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
} GPIO_RegDef_t;

// GPIO peripheral definitions
#define GPIOA					((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*) GPIOH_BASEADDR)

// RCC peripheral register definition structure
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t reserved1[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t reserved2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t reserved3[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t reserved4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t reserved5[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t reserved6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t reserved7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t reserved8;
	__vo uint32_t DCKCFGR;
} RCC_RegDef_t;

// RCC peripheral definition
#define RCC 					((RCC_RegDef_t*) RCC_BASEADDR)

// GPIOx clock enable macros
#define GPIOA_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 7))

// GPIOx clock disable macros
#define GPIOA_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 7))

// GPIOx port reset macros
#define GPIOA_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 0)); (RCC -> AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 1)); (RCC -> AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 2)); (RCC -> AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 3)); (RCC -> AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 4)); (RCC -> AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOH_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 7)); (RCC -> AHB1RSTR &= ~(1 << 7));}while(0)

// Generic macros
#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

#include "stm32f401xx_gpio_driver.h"

// SPI register definition
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

// SPI peripherals
#define SPI1					((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*) SPI4_BASEADDR)

// SPIx clock enable macros
#define SPI1_PCLK_EN()			(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC -> APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC -> APB2ENR |= (1 << 13))

// SPIx clock disable macros
#define SPI1_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 13))

// SPIx port reset macros
#define SPI1_REG_RESET()		do {(RCC -> APB2RSTR |= (1 << 12)); (RCC -> APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 14)); (RCC -> APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 15)); (RCC -> APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_REG_RESET()		do {(RCC -> APB2RSTR |= (1 << 13)); (RCC -> APB2RSTR &= ~(1 << 13));}while(0)

/*
 * SPI peripheral bit position macros
 */
// CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

// SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8



#include "stm32f401xx_spi_driver.h"


#endif /* INC_STM32F401XX_H_ */
