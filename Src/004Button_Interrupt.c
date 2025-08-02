/*
 * 001ledtoggle.c
 *
 *  Created on: Jul 24, 2025
 *      Author: Harshal
 */

#include "stm32f407xx.h"
#include <string.h>
#define HIGH				1
#define LOW					0
#define BTN_PRESSED			LOW

void delay(void)
{
	for(int i=0;i<500000/2;i++);
}
int main(void)
{
#if 1
	GPIO_Handle_t GpioLed,GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));

	GpioLed.pGPIOx=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);


	//this is btn configuration
	GpioBtn.pGPIOx=GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIORITY15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


#endif


	while(1);
	return 0;
}
void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}

