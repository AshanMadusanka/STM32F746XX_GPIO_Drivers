/*
 * stm32f746xx_gpio_driver.h
 *
 *  Created on: Oct 20, 2023
 *      Author: Ashan
 */

#ifndef INC_STM32F746XX_GPIO_DRIVER_H_
#define INC_STM32F746XX_GPIO_DRIVER_H_

#include "stm32f746xx.h"



/*This is Configuration structure for a GPIO Pin*/

typedef struct{

	uint16_t GPIO_PinNumber;
	uint16_t GPIO_PinMode;
	uint16_t GPIO_PinSpeed;
	uint16_t GPIO_PinPuPdControl;
	uint16_t GPIO_PinOPType;
	uint16_t GPIO_PinAltFunMode;


}GPIO_PinConfig_t;


/*This is a Handle structure for a GPIO Pin*/

typedef struct{

	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


// GPIO Possible modes

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT 	6

// GPIO Pin Numbers

#define GPIO_PIN_0   0
#define GPIO_PIN_1   1
#define GPIO_PIN_2   2
#define GPIO_PIN_3   3
#define GPIO_PIN_4   4
#define GPIO_PIN_5   5
#define GPIO_PIN_6   6
#define GPIO_PIN_7   7
#define GPIO_PIN_8   8
#define GPIO_PIN_9   9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15
// GPIO OUTPUT Type Possible modes

#define GPIO_OP_TYPE_PP	0
#define GPIO_OP_TYPE_OD	1


// GPIO OUTPUT Speed Possible modes

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

// GPIO Pull up pull down configuration

#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2


/********************************************************************
 * 						APIs Supported by this driver
 *
 * *******************************************************************/

/*Peripheral Clock Setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*GPIO Init-DeInit*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*GPIO Read Write*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ Configuration and ISR Handling*/

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t PinNumber, uint32_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F746XX_GPIO_DRIVER_H_ */
