/*
 * 001LED_Toggle.c
 *
 *  Created on: Oct 21, 2023
 *      Author: Ashan
 */
#include "stm32f746xx.h"
#include <string.h>
#define HIGH			1
#define BUTTON_PREESED HIGH

void delay(){

	for(uint32_t i=0; i<500000/2; i++);


}

int main(void){


	GPIO_Handle_t GpioLed, GpioButton ,GpioLed2;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioLed2,0,sizeof(GpioLed2));
	memset(&GpioButton,0,sizeof(GpioButton));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_3;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// LED 2

	GpioLed2.pGPIOx = GPIOB;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_0;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioLed2);
	GPIO_Init(&GpioButton);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);




while(1){

	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_0);
	delay();
	delay();



}

}



void EXTI0_IRQHandler(void){

	delay();
	GPIO_IRQHandling( GPIO_PIN_0);

	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_3);

}
