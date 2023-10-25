/*
 * stm32f746xx_gpio_drivers.c
 *
 *  Created on: Oct 20, 2023
 *      Author: Ashan
 */

#include "stm32f746xx.h"
#include "stm32f746xx_gpio_driver.h"

/*Peripheral Clock Setup*/

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){

        if(pGPIOx == GPIOA){

            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB){

            GPIOB_PCLK_EN();
        }
         else if(pGPIOx == GPIOC){

            GPIOC_PCLK_EN();
        }
         else if(pGPIOx == GPIOD){

            GPIOD_PCLK_EN();
        }
         else if(pGPIOx == GPIOE){

            GPIOE_PCLK_EN();
        }
         else if(pGPIOx == GPIOF){

            GPIOF_PCLK_EN();
        }
        else if(pGPIOx == GPIOG){

            GPIOG_PCLK_EN();
        }
         else if(pGPIOx == GPIOH){

            GPIOH_PCLK_EN();
        }
         else if(pGPIOx == GPIOI){

            GPIOI_PCLK_EN();
        }
         else if(pGPIOx == GPIOJ){

            GPIOJ_PCLK_EN();
        }
         else if(pGPIOx == GPIOK){

            GPIOK_PCLK_EN();
        }



	}
    else{
        if(pGPIOx == GPIOA){

            GPIOA_PCLK_DI();
        }
        else if(pGPIOx == GPIOB){

            GPIOB_PCLK_DI();
        }
         else if(pGPIOx == GPIOC){

            GPIOC_PCLK_DI();
        }
         else if(pGPIOx == GPIOD){

            GPIOD_PCLK_DI();
        }
         else if(pGPIOx == GPIOE){

            GPIOE_PCLK_DI();
        }
         else if(pGPIOx == GPIOF){

            GPIOF_PCLK_DI();
        }
        else if(pGPIOx == GPIOG){

            GPIOG_PCLK_DI();
        }
         else if(pGPIOx == GPIOH){

            GPIOH_PCLK_DI();
        }
         else if(pGPIOx == GPIOI){

            GPIOI_PCLK_DI();
        }
         else if(pGPIOx == GPIOJ){

            GPIOJ_PCLK_DI();
        }
         else if(pGPIOx == GPIOK){

            GPIOK_PCLK_DI();
        }


    }

}

/*GPIO Init-DeInit*/
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){


		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else{ // Interrup mode

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//Configure FTSR


			EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |=  ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing RSTR

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//Configure RTSR

			EXTI->RTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |=  ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing FTSR

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){

			//Configure FTSR & RTSR both
			EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

	// 2) Configure GPIO port selection in SYSCFG EXTICR

		uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint32_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode <<(temp2 *4);

	//	3) Enable iterrupt delivery using IMR

		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);




	}
	temp = 0;

	// Config Speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->OSPEEDR |= temp;


	// Config pupd

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->PUPDR |= temp;


	// Config OUTPUT Type

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->OTYPER |= temp;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint8_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~ (0xF <<(4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4 * temp2);
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

    if(pGPIOx == GPIOA){

        GPIOA_REG_RESET();
    }
    else if(pGPIOx == GPIOB){

    	GPIOB_REG_RESET();
    }
     else if(pGPIOx == GPIOC){

    	 GPIOC_REG_RESET();
    }
     else if(pGPIOx == GPIOD){

    	 GPIOD_REG_RESET();
    }
     else if(pGPIOx == GPIOE){

    	 GPIOE_REG_RESET();
    }
     else if(pGPIOx == GPIOF){

    	 GPIOF_REG_RESET();
    }
    else if(pGPIOx == GPIOG){

    	GPIOF_REG_RESET();
    }
     else if(pGPIOx == GPIOH){

    	 GPIOH_REG_RESET();
    }
     else if(pGPIOx == GPIOI){

    	 GPIOI_REG_RESET();
    }
     else if(pGPIOx == GPIOJ){

    	 GPIOJ_REG_RESET();
    }
     else if(pGPIOx == GPIOK){

    	 GPIOK_REG_RESET();
    }


}

/*GPIO Read Write*/
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001 );

	return value;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;

	value = (uint16_t)pGPIOx->IDR ;

	return value;


}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(GPIO_PIN_SET == Value){

		pGPIOx->ODR |= (1 << PinNumber);

	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);

	}


}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;


}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);


}

/*IRQ Configuration and ISR Handling*/

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){

if(EnorDi == ENABLE){  // ISERx Registers Enable

	if(IRQNumber <= 31){

		*NVIC_ISER0 |= (1<< IRQNumber);
	}else if(IRQNumber > 31 && IRQNumber < 64){

		*NVIC_ISER1 |= (1<< IRQNumber % 32);

	}else if (IRQNumber >= 64 && IRQNumber < 96){

		*NVIC_ISER2 |= (1<< IRQNumber % 64);
	}



}else {  // ICERx Registers Enable

	if(IRQNumber <= 31){

			*NVIC_ICER0 |= (1<< IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){

			*NVIC_ICER1 |= (1<< IRQNumber % 32);

		}else if (IRQNumber >= 64 && IRQNumber < 96){

			*NVIC_ICER2 |= (1<< IRQNumber % 64);
		}



}
}

void GPIO_IRQPriorityConfig(uint8_t PinNumber, uint32_t IRQPriority){

 uint8_t iprx = IRQPriority /4;
 uint8_t iprx_section = IRQPriority % 4;

  uint8_t shift_amount = ( 8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

  *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority <<shift_amount);


}

void GPIO_IRQHandling(uint8_t PinNumber){

	if(EXTI->PR & (1<< PinNumber)){


		EXTI->PR |= (1<<PinNumber);
	}

}
