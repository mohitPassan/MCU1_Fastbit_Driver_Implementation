/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Sep 6, 2025
 *      Author: mohitpassan
 */

#include "../Inc/stm32f401xx_spi_driver.h"

/*
 * Peripheral clock control
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		} else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	} else {
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		} else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		} else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		} else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*
 * SPI_Init implementation
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Taking a temporary CR1 variable
	uint32_t cr1_temp = 0;

	// Setting device mode
	cr1_temp |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// Setting bus config
	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// Clear the BIDI mode
		cr1_temp &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Set the BIDI mode
		cr1_temp |= 1 << SPI_CR1_BIDIMODE;
	} else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// Clear the BIDI mode and set the RXOnly bit
		cr1_temp &= ~(1 << SPI_CR1_BIDIMODE);
		cr1_temp |= 1 << SPI_CR1_RXONLY;
	}

	// Setting DFF
	cr1_temp |= pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF;

	// Setting CPHA
	cr1_temp |= pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA;

	// Setting CPOL
	cr1_temp |= pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL;

	// Setting SSM
	cr1_temp |= pSPIHandle->SPI_PinConfig.SPI_SSM << SPI_CR1_SSM;

	// Setting speed
	cr1_temp |= pSPIHandle->SPI_PinConfig.SPI_Speed << SPI_CR1_BR;

	// Setting the CR1 register using the temporary register
	pSPIHandle->pSPIx->CR1 = cr1_temp;
}


