/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Sep 6, 2025
 *      Author: mohitpassan
 */

#include "../Inc/stm32f401xx_spi_driver.h"

/*
 * Function to get the status of the flag
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx -> SR & flagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

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
	// 0. Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

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

/*
 * SPI de-init implementation
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	} else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	} else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	} else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/*
 * SPI send data
 * @param[*pSPIx]		-	base address of the SPI peripheral
 * @param[*pTxBuffer]	-	base address of the data
 * @param[len]			-	length of the data
 * @note				-	This is a blocking call or polling based
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			pSPIx->DR = *(uint16_t*)pTxBuffer;
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		} else
		{
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

/*
 * Enable or disable SPI peripheral
 */
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx -> CR1 |= (1 << SPI_CR1_SPE);
	} else
	{
		pSPIx -> CR1 &= ~(0 << SPI_CR1_SPE);
	}
}


