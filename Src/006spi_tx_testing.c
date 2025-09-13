/*
 * 006spi_tx_testing.c
 *
 *  Created on: Sep 11, 2025
 *      Author: mohitpassan
 */

/*
 * PA4 -> SPI1_NSS
 * PA5 -> SPI1_SCK
 * PA6 -> SPI1_MISO
 * PA7 -> SPI1_MOSI
 * Alternate function mode: 5
 */

#include "stm32f401xx_gpio_driver.h"
#include <string.h>

void SPI1_GPIO_Inits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// PA4 -> SPI1_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_4;
	GPIO_Init(&SPIPins);

	// PA5 -> SPI1_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_5;
	GPIO_Init(&SPIPins);

	// PA6 -> SPI1_MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_6;
	GPIO_Init(&SPIPins);

	// PA7 -> SPI1_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_7;
	GPIO_Init(&SPIPins);
}

void SPI1_Inits(void)
{
	SPI_Handle_t pinHandle;

	pinHandle.pSPIx = SPI1;
	pinHandle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	pinHandle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	pinHandle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	pinHandle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	pinHandle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	pinHandle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN;
	pinHandle.SPI_PinConfig.SPI_Speed = SPI_SCLK_SPEED_DIV2;

	SPI_Init(&pinHandle);
}

int main(void)
{
	char user_data[] = "Hello, world!";

	SPI1_GPIO_Inits();
	SPI1_Inits();

	SPI_Peripheral_Control(SPI1, ENABLE);
	SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));
	return 0;
}
