/*
 * stm32f746xx.h
 *
 *  Created on: Oct 20, 2023
 *      Author: Ashan
 */

#ifndef INC_STM32F746XX_H_
#define INC_STM32F746XX_H_

#include <stdint.h>

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20010000U
#define SRAM					SRAM1_BASEADDR
#define SRAM2_BASEADDR			0x2004C000U
#define ROM_BASEADDR			0x1FF00000U


#define __vo volatile

/*###################################### Processor Specific Details #########################################*/

/*Arm Cortex Mx Processor NVIC ISERx Register Addresses*/

#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

/*Arm Cortex Mx Processor NVIC ICERx Register Addresses*/

#define NVIC_ICER0			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*Arm Cortex Mx Processor NVIC PR Register Addresses*/
#define NVIC_PR_BASEADDR			((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4

/*
 * AHBx and APBx Peripherals Base addresses
*/
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define AHB3PERIPH_BASEADDR		0xA0000000U

/*
 * Peripherals base addresses which are hanging on AHB1 Bus
*/

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)


#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Peripherals base addresses which are hanging on APB1 Bus
*/

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)
#define I2C4_BASEADDR			(APB1PERIPH_BASEADDR + 0x6000)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR			(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR			(APB1PERIPH_BASEADDR + 0x7C00)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)

/*
 * Peripherals base addresses which are hanging on APB2 Bus
*/

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR			(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR			(APB2PERIPH_BASEADDR + 0x5400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

/*********************************** Peripheral register definition structure***************************/

typedef struct{

	__vo uint32_t MODER;				/*GPIO port mode register                      Address offset: 0x00*/
	__vo uint32_t OTYPER;				/*GPIO port output type register               Address offset: 0x04*/
	__vo uint32_t OSPEEDR;				/*GPIO port output speed register              Address offset: 0x08*/
	__vo uint32_t PUPDR;				/*GPIO port pull-up/pull-down register         Address offset: 0x0C*/
	__vo uint32_t IDR;					/*GPIO port input data register                Address offset: 0x10*/
	__vo uint32_t ODR;					/*GPIO port output data register               Address offset: 0x14*/
	__vo uint32_t BSRR;					/*GPIO port bit set/reset register             Address offset: 0x18*/
	__vo uint32_t LCKR;					/*GPIO port configuration lock register        Address offset: 0x1C  */
	__vo uint32_t AFR[2];				/*GPIO alternate function low register AFRL[0] Address offset: 0x20 / GPIO alternate function high register AFRL[1]*  0x24*/

}GPIO_RegDef_t;

/************************************ RCC Register definition structur*******************************************/

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
		 uint32_t Reserved4;

	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
		 uint32_t Reserved[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
		 uint32_t Reserved5;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
		 uint32_t Reserved1[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
		 uint32_t Reserved6;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	     uint32_t Reserved2[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
		 uint32_t Reserved3[2];

	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SC;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR1;
	__vo uint32_t DCKCFGR2;

}RCC_RegDef_t;

/************************************ RCC Register definition structur*******************************************/


typedef struct{

	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;


/************************************ SYSCONFIG Register definition structur*******************************************/


typedef struct{

	__vo uint32_t MEMRMP; // 0x00
	__vo uint32_t PMC;	// 0x04
	__vo uint32_t EXTICR[4]; // 0x08 - 0x14
		 uint32_t Reserved[2]; // 0x18, 0x1C
	__vo uint32_t CMPCR;	// 0x20


}SYSCFG_RegDef_t;



/*Peripheral Definition (Peripherals base address typecasted to xxxReg_def)*/

#define GPIOA  (GPIO_RegDef_t*)GPIOA_BASEADDR
#define GPIOB  (GPIO_RegDef_t*)GPIOB_BASEADDR
#define GPIOC  (GPIO_RegDef_t*)GPIOC_BASEADDR
#define GPIOD  (GPIO_RegDef_t*)GPIOD_BASEADDR
#define GPIOE  (GPIO_RegDef_t*)GPIOE_BASEADDR
#define GPIOF  (GPIO_RegDef_t*)GPIOF_BASEADDR
#define GPIOG  (GPIO_RegDef_t*)GPIOG_BASEADDR
#define GPIOH  (GPIO_RegDef_t*)GPIOH_BASEADDR
#define GPIOI  (GPIO_RegDef_t*)GPIOI_BASEADDR
#define GPIOJ  (GPIO_RegDef_t*)GPIOJ_BASEADDR
#define GPIOK  (GPIO_RegDef_t*)GPIOK_BASEADDR

#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI    ((EXTI_RegDef_t*)EXTI_BASEADDR)



#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*Clock enable Macros for GPIOx*/


#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()			(RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()			(RCC->AHB1ENR |= (1<<10))

/*Clock enable Macros for I2Cx*/

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1<<23))
#define I2C4_PCLK_EN()			(RCC->APB1ENR |= (1<<24))

/*Clock enable Macros for SPIx*/

#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()			(RCC->APB2ENR |= (1<<20))
#define SPI6_PCLK_EN()			(RCC->APB2ENR |= (1<<21))

/*Clock enable Macros for USARTx*/

#define USART1_PCLK_EN()		(RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN()		(RCC->APB2ENR |=(1<<18))
#define USART6_PCLK_EN()		(RCC->APB2ENR |=(1<<5))

/*Clock enable Macros for UARTx*/

#define UART4_PCLK_EN()		    (RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |=(1<<20))
#define UART7_PCLK_EN()			(RCC->APB1ENR |=(1<<30))
#define UART8_PCLK_EN()			(RCC->APB1ENR |=(1<<31))

/*Clock enable Macros for SYSCFG*/

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |=(1<<14))

/*===============================Disable CLK Macros===================================================*/

/*Clock Disable Macros for GPIOx*/

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<8))
#define GPIOJ_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<9))
#define GPIOK_PCLK_DI()			(RCC->AHB1ENR &= ~ (1<<10))

/*Clock Disable Macros for I2Cx*/

#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~ (1<<21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~ (1<<22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~ (1<<23))
#define I2C4_PCLK_DI()			(RCC->APB1ENR &= ~ (1<<24))

/*Clock Disable Macros for SPIx*/

#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~ (1<<12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~ (1<<14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~ (1<<15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~ (1<<13))
#define SPI5_PCLK_DI()			(RCC->APB2ENR &= ~ (1<<20))
#define SPI6_PCLK_DI()			(RCC->APB2ENR &= ~ (1<<21))

/*Clock Disable Macros for USARTx*/

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB2ENR &= ~(1<<18))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<5))

/*Clock Disable Macros for UARTx*/

#define UART4_PCLK_DI()		    (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1<<20))
#define UART7_PCLK_DI()			(RCC->APB1ENR &= ~(1<<30))
#define UART8_PCLK_DI()			(RCC->APB1ENR &= ~(1<<31))

/*Clock Disable Macros for SYSCFG*/

#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1<<14))

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET


#define GPIO_BASEADDR_TO_CODE(x)  (  (x == GPIOA) ? 0 : \
                                   (x == GPIOB) ? 1 : \
                                   (x == GPIOC) ? 2 : \
                                   (x == GPIOD) ? 3 : \
                                   (x == GPIOE) ? 4 : \
                                   (x == GPIOF) ? 5 : \
                                   (x == GPIOG) ? 6 : \
                                   (x == GPIOH) ? 7 : \
                                   (x == GPIOI) ? 8 : \
                                   (x == GPIOJ) ? 9 : \
                                   (x == GPIOK) ? 10 : 0 )


// IRQ Number

#define IRQ_NO_EXTI0   		6
#define IRQ_NO_EXTI1   		7
#define IRQ_NO_EXTI2   		8
#define IRQ_NO_EXTI3   		9
#define IRQ_NO_EXTI4   		10
#define IRQ_NO_EXTI9_5   	23
#define IRQ_NO_EXTI15_10   	40


#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI15		15


// Macros for reset GPIOx Peripherals

#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)
#define GPIOJ_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<9)); (RCC->AHB1RSTR &= ~(1<<9));}while(0)
#define GPIOK_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<10)); (RCC->AHB1RSTR &= ~(1<<10));}while(0)

#include "stm32f746xx_gpio_driver.h"


#endif /* INC_STM32F746XX_H_ */
