/*
 * 001LED_Toggle.c
 *
 *  Created on: Oct 21, 2023
 *      Author: Ashan
 */
#include "stm32f746xx.h"

#define HIGH			1
#define BUTTON_PREESED HIGH

void delay(){

	for(uint32_t i=0; i<1000000; i++);


}

int main(void){


	GPIO_Handle_t GpioLed, GpioButton;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_3;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);
	;

while(1){

	if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_0)== BUTTON_PREESED)
	{
	delay();
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_3);

	}
}

}
