/*
 * stm32f407xx.h
 *
 *  Created on: Jul 20, 2025
 *      Author: Harshal
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

#define _vo							volatile
/******************************************* Processor Specific details************************************************/
/*** ARM cortex Mx processor NVIC ISERx register address ***/
#define NVIC_ISER0					((_vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((_vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((_vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((_vo uint32_t*)0xE000E10C)


/*** ARM cortex Mx processor NVIC ICERx register address ***/
#define NVIC_ICER0					((_vo uint32_t*)0XE000E180)
#define NVIC_ICER1					((_vo uint32_t*)0XE000E184)
#define NVIC_ICER2					((_vo uint32_t*)0XE000E188)
#define NVIC_ICER3					((_vo uint32_t*)0XE000E18C)

/*** ARM cortex Mx processor NVIC IPRx register address ***/
#define NVIC_PR_BASE_ADDR			((_vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED		4

/*****Base addresses of flash and SRAM memories   ******/
#define FLASB_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x2001C000U
#define ROM							0x1FFF0000U
#define SRAM 						SRAM1_BASEADDR


/*****Base addresses AHBx and APBx ******/
#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x50000000U

/*****Base addresses of peripherals which are hanging on AHB1 bus ******/
#define GPIOA_BASEADDR				(AHB1PERIPH_BASE+0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASE+0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE+0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE+0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE+0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASE+0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASE+0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE+0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASE+0x2000)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASE+0x2400)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASE+0x2800)
#define RCC_BASEADDR				(AHB1PERIPH_BASE+0x3800)

/*****Base addresses of peripherals which are hanging on APB1 bus ******/

#define I2C1_BASEADDR				(APB1PERIPH_BASE+0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASE+0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASE+0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASE+0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASE+0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASE+0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASE+0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASE+0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASE+0x5000)

/*****Base addresses of peripherals which are hanging on APB2 bus ******/
#define EXTI_BASEADDR				(APB2PERIPH_BASE+0x3C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASE+0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASE+0x3400)
#define USART1_BASEADDR				(APB2PERIPH_BASE+0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASE+0x1400)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE+0x3800)


/*****Peripheral register definition structures ******/
/***************Registers of peripheral are specific to MCU ***********/

typedef struct
{
	_vo uint32_t MODER;
	_vo uint32_t OTYPER;
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFR[2];
}GPIO_RegDef_t;


typedef struct
{
	_vo uint32_t CR;
	_vo uint32_t PLLCFGR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t AHB1RSTR;
	_vo uint32_t AHB2RSTR;
	_vo uint32_t AHB3RSTR;
	_vo uint32_t RESERVED0;
	_vo uint32_t APB1RSTR;
	_vo uint32_t APB2RSTR;
	_vo uint32_t RESERVED1;
	_vo uint32_t RESERVED2;
	_vo uint32_t AHB1ENR;
	_vo uint32_t AHB2ENR;
	_vo uint32_t AHB3ENR;
	_vo uint32_t RESERVED3;
	_vo uint32_t APB1ENR;
	_vo uint32_t APB2ENR;
	_vo uint32_t RESERVED4;
	_vo uint32_t RESERVED5;
	_vo uint32_t AHB1LPENR;
	_vo uint32_t AHB2LPENR;
	_vo uint32_t AHB3LPENR;
	_vo uint32_t RESERVED6;
	_vo uint32_t APB1LPENR;
	_vo uint32_t APB2LPENR;
	_vo uint32_t RESERVED7;
	_vo uint32_t RESERVED8;
	_vo uint32_t RCC_BDCR;
	_vo uint32_t RCC_CSR;
	_vo uint32_t RESERVED9;
	_vo uint32_t RESERVED10;
	_vo uint32_t RCC_SSCGR;
	_vo uint32_t PLLI2SCFGR;
	_vo uint32_t PLLSAICFGR;
	_vo uint32_t DCKCFGR;
}RCC_RegDef_t;


/***Peripheral register definition structure for EXTI ***/

typedef struct
{
	_vo uint32_t IMR;
	_vo uint32_t EMR;
	_vo uint32_t RTSR;
	_vo uint32_t FTSR;
	_vo uint32_t SWIER;
	_vo uint32_t PR;
}EXTI_RegDef_t;


/******* peripheral register definition structure for SPI *******/

typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t CRCPR;
	_vo uint32_t RXCRCR;
	_vo uint32_t TXCRCR;
	_vo uint32_t I2SCFGR;
	_vo uint32_t I2SPR;
}SPI_RegDef_t;


/*** Peripheral register definition structure for SYSCFG ***/

typedef struct
{
	_vo uint32_t MEMRMP;
	_vo uint32_t PMC;
	_vo uint32_t EXTICR[4];
	uint32_t     RESERVED1[2];
	_vo uint32_t CMPCR;
	uint32_t     RESERVED2[2];
	_vo uint32_t CFGR;

}SYSCFG_RegDef_t;

/***********Peripheral definitions(Peripheral base addresses type casted to xxx_RegDef_t  ************/

#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ						((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK						((GPIO_RegDef_t*)GPIOK_BASEADDR)


#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4						((SPI_RegDef_t*)SPI4_BASEADDR)
/***Clock enable macros for GPIOx peripherals***/

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()	(RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()	(RCC->AHB1ENR |= (1<<10))

/** Clock enable macros for I2C peripherals **/
#define I2C1_PCLK_EN()  	(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()  	(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()  	(RCC->APB1ENR |= (1<<23))


/** Clock enable macros for SPI peripherals **/
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()  	(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()  	(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1<<13))
/** Clock enable macros for USART peripherals **/
#define USART1_PCLK_EN()  	(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()  	(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()  	(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()  	(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()  	(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()  	(RCC->APB2ENR |= (1<<5))

/** Clock enable macros for SYSCFG peripheral **/
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))


/***Clock disable macros for GPIOx peripherals***/

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<10))

/** Clock disable macros for I2Cx peripherals **/
#define I2C1_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<23))


/** Clock disable macros for SPIx peripherals **/
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<13))

/** Clock disable macros for USARTx peripherals **/
#define USART1_PCLK_DI()  	(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()  	(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()  	(RCC->APB2ENR &= ~(1<<5))

/** Clock disable macros for SYSCFG peripheral **/
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14))

/** Macros to reset GPIOx peripherals  **/
#define GPIOA_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<1));  (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<2));  (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<3));  (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<4));  (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<5));  (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<6));  (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<7));  (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<8));  (RCC->AHB1RSTR &= ~(1<<8));}while(0)
#define GPIOJ_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<9));  (RCC->AHB1RSTR &= ~(1<<9));}while(0)
#define GPIOK_REG_RESET()	do {(RCC->AHB1RSTR |= (1<<10));  (RCC->AHB1RSTR &= ~(1<<10));}while(0)


/** Macros to reset SPIx peripherals ***/
#define SPI1_REG_RESET()	do {(RCC->APB2RSTR |= (1<<12));  (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()	do {(RCC->APB1RSTR |= (1<<14));  (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()	do {(RCC->APB1RSTR |= (1<<15));  (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()	do {(RCC->APB2RSTR |= (1<<13));  (RCC->APB2RSTR &= ~(1<<13));}while(0)


#define GPIO_BASEADDR_TO_CODE(x)    ((x==GPIOA)?0:\
									(x==GPIOB)?1:\
									(x==GPIOC)?2:\
									(x==GPIOD)?3:\
									(x==GPIOE)?4:\
									(x==GPIOF)?5:\
									(x==GPIOG)?6:\
									(x==GPIOH)?7:\
									(x==GPIOI)?8:\
									(x==GPIOJ)?9:\
									(x==GPIOK)?10:0)


/*** Interrupt request numbers of STM32F407xMCU ***/
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10	    40

/***NVIC Priority levels***/


#define NVIC_IRQ_PRIORITY15     15


//some generic macros
#define ENABLE			1
#define DISABLE			0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET


/*************************** Bit position definitions of SPI peripheral ******************************/
/***BIT position definition of CR1 ***/
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR	 		3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST 	7
#define SPI_CR1_SSI 		8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15


/***BIT position definition of CR2 ***/
#define SPI_CR2_RXDMAEN	 	0
#define SPI_CR2_TXDMAEN	 	1
#define SPI_CR2_SSOE	 	2
#define SPI_CR2_RESERVED1	3
#define SPI_CR2_FRF	 	    4
#define SPI_CR2_ERRIE	 	5
#define SPI_CR2_RXNEIE	 	6
#define SPI_CR2_TXEIE	 	7
#define SPI_CR2_RESERVED2	8

/***BIT position definition of SR ***/
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_CR2_RESERVED	9




#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#endif /* INC_STM32F407XX_H_ */
