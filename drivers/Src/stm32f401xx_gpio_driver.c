/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Jun 18, 2024
 *      Author: mohitpassan
 */
#include "../Inc/stm32f401xx.h"
#include "../Inc/stm32f401xx_gpio_driver.h"
#include <stdint.h>

/*
 * Peripheral clock control
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and de-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0;
	// 1. Configure pin mode
	// Check if the mode is interrupt type or not
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Non-interrupt mode
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle -> pGPIOx -> MODER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle -> pGPIOx -> MODER |= temp;
	} else {
		// Interrupt mode. Will do this later
	}
	temp = 0;

	// 2. Configure speed
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OSPEEDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;
	temp = 0;

	// 3. Configure pupd settings
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> PUPDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;
	temp = 0;

	// 4. Configure output type
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinOpType << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;
	temp = 0;

	// 5. Configure alt functionality
	// Only if the GPIO mode is alt functionality
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// Configure alt functionality
		uint32_t lowerOrHigher, pinOffset;
		lowerOrHigher = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 8;

		uint8_t afrMode = pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode;
		pinOffset = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;

		if(lowerOrHigher == 0) {
			// lower AFR register
			pGPIOHandle -> pGPIOx -> AFRL &= ~(0xF << 4 * pinOffset);
			pGPIOHandle -> pGPIOx -> AFRL |= afrMode << 4 * pinOffset;
		} else {
			// higher AFR register
			pGPIOHandle -> pGPIOx -> AFRH &= ~(0xF << 4 * pinOffset);
			pGPIOHandle -> pGPIOx -> AFRH |= afrMode << 4 * pinOffset;
		}
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*
 * Read and write data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	// We have to read from IDR register
	uint8_t value;
	value = (uint8_t)(pGPIOx -> IDR >> PinNumber) & 0x00000001;
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t)pGPIOx -> IDR;
	return value;
}

void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value) {
	if(value == SET) {
		pGPIOx -> ODR |= (0x1 << PinNumber);
	} else {
		pGPIOx -> ODR &= ~(0x1 << PinNumber);
	}
}

void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx -> ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx -> ODR ^= (0x1 << PinNumber);
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


