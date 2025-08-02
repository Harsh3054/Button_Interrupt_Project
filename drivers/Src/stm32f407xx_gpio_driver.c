/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 22, 2025
 *      Author: Harshal
 */
#include "stm32f407xx_gpio_driver.h"

/******************************************************************************************
    * @fn			-GPIO_PeriClockControl
	* @brief   		-This function enables or disable peripheral clock for given GPIO port
	* @param[in]  	-Base address of GPIO peripheral
	* @param[in] 	-ENABLE or DISABLE macros
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if(pGPIOx==GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else if(pGPIOx==GPIOK)
		{
			GPIOK_PCLK_EN();
		}
		else
		{
			;
		}
	}
	else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_DI();
		}
		else if(pGPIOx==GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}
		else if(pGPIOx==GPIOK)
		{
			GPIOK_PCLK_DI();
		}
		else
		{
			;
		}

	}

}

/******************************************************************************************
    * @fn			-GPIO_Init
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1.configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		//The non interrupt mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode  << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//clear bits
		pGPIOHandle->pGPIOx->MODER |= temp;//set bits

	}
	else
	{
		//Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.configure the FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.configure the RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1.configure FTSR,RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2.configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]=portcode << (temp2 * 4);

		//3.enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	//2.configure the speed.
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//clear bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3.configure the pupd settings
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//clear bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//4.configure the optype.
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clear bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5.configure the alternate functionality mode
	temp=0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
	{
		//configure the alt function register
		uint8_t AFRLorAFRH;
		uint8_t pinNumber;
		AFRLorAFRH=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		pinNumber=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

		pGPIOHandle->pGPIOx->AFR[AFRLorAFRH] &= ~(0xF << (4*pinNumber));
		pGPIOHandle->pGPIOx->AFR[AFRLorAFRH] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*pinNumber));

	}
}

/******************************************************************************************
    * @fn			-GPIO_DeInit
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx==GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx==GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx==GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx==GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else if(pGPIOx==GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else if(pGPIOx==GPIOK)
	{
		GPIOK_REG_RESET();
	}
	else
	{
		;
	}
}

/******************************************************************************************
    * @fn			-GPIO_ReadFromInputPin
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-0 or 1
	* @Note			-None
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	return value;
}

/******************************************************************************************
    * @fn			-GPIO_ReadFromInputPort
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-
	* @Note			-None
*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value =(uint16_t)(pGPIOx->IDR);
	return value;
}

/******************************************************************************************
    * @fn			-GPIO_WriteToOutputPin
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-
	* @Note			-None
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value)
{
	if(value==GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}


/******************************************************************************************
    * @fn			-GPIO_WriteToOutputPort
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-
	* @Note			-None
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR = value;
}

/******************************************************************************************
    * @fn			-GPIO_ToggleOutputPin
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-
	* @Note			-None
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/******************************************************************************************
    * @fn			-GPIO_IRQConfig
	* @brief   		-IRQ Configuration
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-
	* @Note			-None
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{

	if(EnorDi ==ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1<<(IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1<<(IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber<64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1<<(IRQNumber % 32));
		}
		else if(IRQNumber >=64 && IRQNumber<96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1<<(IRQNumber % 64));
		}
	}
}

/******************************************************************************************
    * @fn			-GPIO_IRQPriorityConfig
	* @brief   		-IRQ Priority
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-
	* @Note			-None
*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1.First lets find out ipr register
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	uint8_t shift_amount=(8 * iprx_section)+ (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+(iprx)) |= (IRQPriority << shift_amount );
}


/******************************************************************************************
    * @fn			-GPIO_IRQHandling
	* @brief   		-ISR handling
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-
	* @Note			-None
*/

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//1.clear the EXTI PR register corresponding to the PIN number

	if(EXTI->PR & (1<<PinNumber))
	{
		//clear
		EXTI->PR |= (1<<PinNumber);
	}

}


