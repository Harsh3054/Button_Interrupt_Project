/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Harshal
 */
#include "stm32f407xx_spi_driver.h"

/******************************************************************************************
    * @fn			-SPI_PeriClockControl
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
		{
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx==SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx==SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if(pSPIx==SPI4)
			{
				SPI4_PCLK_EN();
			}
			else
			{
				;
			}
		}
		else
		{
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if(pSPIx==SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx==SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if(pSPIx==SPI4)
			{
				SPI4_PCLK_DI();
			}
			else
			{
				;
			}

		}
}

/******************************************************************************************
    * @fn			-SPI_Init
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register
	uint32_t tempreg=0;

	//1.configure the device mode.
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.configure the bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be SET
		tempreg |= (1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{

		//BIDI mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXonly bit must be SET
		tempreg |= (1<<SPI_CR1_RXONLY);
	}

	//3.configure the SPI serial clock speed(baud rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;


	//4.configure the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	//5.configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//6.configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/******************************************************************************************
    * @fn			-SPI_DeInit
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx==SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx==SPI4)
	{
		SPI4_REG_RESET();
	}
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName) return FLAG_SET;
	return FLAG_RESET;
}


/******************************************************************************************
    * @fn			-SPI_SendData
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-This is blocking call
*/
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len>0)
	{
		//1.wait until TXE is SET.
		while((SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET));


		//2.check the DFF bit in CR1
		if(( pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1.load the data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len-=2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len-=1;
			pTxBuffer++;
		}
	}
}

/******************************************************************************************
    * @fn			-SPI_ReceiveData
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{



}

/******************************************************************************************
    * @fn			-SPI_PeripheralControl
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE) ;
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE) ;
	}
}

/******************************************************************************************
    * @fn			-SPI_SSIConfig
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI) ;
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI) ;
	}
}



/******************************************************************************************
    * @fn			-SPI_SSOEConfig
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE) ;
	}
	else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE) ;
	}
}
/******************************************************************************************
    * @fn			-SPI_IRQInterruptConfig
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{


}

/******************************************************************************************
    * @fn			-SPI_IRQHandling
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{


}

/******************************************************************************************
    * @fn			-SPI_IRQPriorityConfig
	* @brief   		-
	* @param[in]  	-
	* @param[in] 	-
	* @param[in]
	* @retval		-None
	* @Note			-None
*/
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{


}
