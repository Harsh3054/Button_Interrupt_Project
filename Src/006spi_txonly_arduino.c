/*
 * 006spi_txonly_arduino.c
 *
 *  Created on: Aug 2, 2025
 *      Author: Harshal
 */
#include "stm32f407xx.h"
#include <string.h>
/*PB15->SPI2_MOSI
PB14->SPI2_MISO
PB13->SPI2_SCLK
PB12->SPI2_NSS
ALT function mode->5*/

void delay(void)
{
	for(int i=0;i<500000/2;i++);
}


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{

	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;//generates SCLK of 8MHz
	SPI2handle.SPI_Config.SPI_DFF=SPI_DIFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA=SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM=SPI_SSM_DI;//Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);


}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx=GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}


int main(void)
{
	char user_data[]="Hello world";

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 PINS
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable
	 * The NSS pin is automatically managed by the hardware.
	 * i.e. when SPE=1,NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)));

		delay();
		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//first send length information
		uint8_t datalen=strlen(user_data);
		SPI_SendData(SPI2, &datalen, 1);

		//to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeripheralControl(SPI2,DISABLE);
	}

	return 0;
}


