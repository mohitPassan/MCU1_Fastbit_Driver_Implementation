/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: Jun 18, 2024
 *      Author: mohitpassan
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include <stdint.h>

#include "stm32f401xx.h"

// GPIO Pin config
typedef struct {
	uint8_t GPIO_PinNumber;			/* possible values: @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;			/* possible values: @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* possible values: @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;	/* possible values: @GPIO_PIN_PUPD */
	uint8_t GPIO_PinOpType;			/* possible values: @GPIO_PIN_OP_TYPE */
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

// GPIO handle structure
typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * @GPIO_PIN_MODES
 * GPIO possible modes
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO possible output modes
 */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*
 * @GPIO_PIN_SPEED
 * GPIO possible speed modes
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PIN_PUPD
 * GPIO PUPD control
 */

#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NUMBER_0		0
#define GPIO_PIN_NUMBER_1		1
#define GPIO_PIN_NUMBER_2		2
#define GPIO_PIN_NUMBER_3		3
#define GPIO_PIN_NUMBER_4		4
#define GPIO_PIN_NUMBER_5		5
#define GPIO_PIN_NUMBER_6		6
#define GPIO_PIN_NUMBER_7		7
#define GPIO_PIN_NUMBER_8		8
#define GPIO_PIN_NUMBER_9		9
#define GPIO_PIN_NUMBER_10		10
#define GPIO_PIN_NUMBER_11		11
#define GPIO_PIN_NUMBER_12		12
#define GPIO_PIN_NUMBER_13		13
#define GPIO_PIN_NUMBER_14		14

/*******************************
 * API supported by the driver
 *******************************/

/*
 * Peripheral clock control
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and de-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and write data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
