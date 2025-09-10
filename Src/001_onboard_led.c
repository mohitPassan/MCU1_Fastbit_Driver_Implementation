#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f401xx.h"

void delay()
{
	for(int i=0; i<1000000; i++)
	{}
}

int main(void)
{
	GPIO_Handle_t gpioHandle;

	// Setting up the GPIO handle
	gpioHandle.pGPIOx = GPIOA;
	gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_5;
	gpioHandle.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	// Enabling the clock
	GPIO_PeriClockControl(GPIOA, ENABLE);

	// Initialise the GPIO register
	GPIO_Init(&gpioHandle);

    /* Loop forever */
	for(;;)
	{
		// Toggle the pin
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUMBER_5);
		delay();
	}
}
