/*
 * stm32f411re.h
 *
 *  Created on: Dec 13, 2023
 *      Author: irshan
 */

#ifndef INC_STM32F411RE_H_
#define INC_STM32F411RE_H_
#include<stdint.h>

#define __vo volatile

/******************************************************************************************************************
* 									Processor Specific Details
******************************************************************************************************************/
/*
 *  				           ARM Cortex Mx Processor NVIC ISERx Register Addresses
*/

#define NVIC_ISER0				(( __vo uint32_t *) 0xE000E100 )
#define NVIC_ISER1				(( __vo uint32_t *) 0xE000E104 )
#define NVIC_ISER2				(( __vo uint32_t *) 0xE000E108 )
#define NVIC_ISER3				(( __vo uint32_t *) 0xE000E10C )



/*
 *  				           ARM Cortex Mx Processor NVIC ICERx Register Addresses
*/
#define NVIC_ICER0				(( __vo uint32_t *)0xE000E180)
#define NVIC_ICER1				(( __vo uint32_t *)0xE000E184)
#define NVIC_ICER2				(( __vo uint32_t *)0xE000E188)
#define NVIC_ICER3				(( __vo uint32_t *)0xE000E18C)


/*
 *  				           ARM Cortex Mx Processor NVIC ICERx Register Addresses
*/
#define NVIC_PR_BASE_ADDR				(( __vo uint32_t *)0xE000E400)

/* its different for every MCU  */
#define NO_PR_BITS_IMPLEMENTATION 			4







/*
 * BASE ADDRESSES OF FLASH AND SRAM MEMORIES (1)
 */

#define FLASH_BASEADDR  				0x08000000U
#define SRAM1_BASEADDR  				0x20000000U
#define SRAM 							SRAM1_BASEADDR  /*SRAM1 IS CALLED AS MAIN SRAM*/
#define ROM								0x1FFF0000U /*SYSTEM MEMORY IS ROM*/
/*#define SRAM2_BASEADDR*/


/*
 *  AHBx AND APBx BUS PERIPHERAL BASE ADDRESSES (2)
 */
#define PERIPH_BASE 					0x40000000U
#define APB1PERIPH_BASE 				PERIPH_BASE
#define APB2PERIPH_BASE 				0x40010000U
#define AHB1PERIPH_BASE 				0x40020000U
#define AHB2PERIPH_BASE 				0x50000000U

/*
 *  BASE ADDRESSES OF  PERIPHERAL HANGING IN AHB1 BUS (3)
 */
#define GPIOA_BASEADDR					(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASE + 0x1C00)
#define CRC_BASEADDR					(AHB1PERIPH_BASE + 0x3000)
#define RCC_BASEADDR					(AHB1PERIPH_BASE + 0x3800)
#define FLASH_INTERF_REG_BASEADDR		(AHB1PERIPH_BASE + 0x3C00)
#define DMA1_BASEADDR					(AHB1PERIPH_BASE + 0x6000)
#define DMA2_BASEADDR					(AHB1PERIPH_BASE + 0x6400)



/*
 *  BASE ADDRESSES OF  PERIPHERAL HANGING IN AHB2 BUS (4)
 */

#define USB_OTG_FS_BASEADDR				(AHB2PERIPH_BASE + 0x0000)




/*
 *  BASE ADDRESSES OF  PERIPHERAL HANGING IN APB1 BUS (5)
 */
#define TIM2_BASEADDR					(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR					(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR					(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR					(APB1PERIPH_BASE + 0x0C00)
#define RTC_BKP_REG_BASEADDR			(APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR					(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR					(APB1PERIPH_BASE + 0x3000)
#define I2S2EXT_BASEADDR				(APB1PERIPH_BASE + 0x3400)
#define SPI2_I2S2_BASEADDR				(APB1PERIPH_BASE + 0x3800)
#define SPI3_I2S3_BASEADDR				(APB1PERIPH_BASE + 0x3C00)
#define I2S3EXT_BASEADDR				(APB1PERIPH_BASE + 0x4000)
#define USART2_BASEADDR					(APB1PERIPH_BASE + 0x4400)
#define I2C1_BASEADDR					(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASE + 0x5C00)
#define PWR_BASEADDR					(APB1PERIPH_BASE + 0x7000)



/*
 *  BASE ADDRESSES OF  PERIPHERAL HANGING IN APB2 BUS (6)
 */

#define TIM1_BASEADDR					(APB2PERIPH_BASE + 0x0000)
#define USART1_BASEADDR					(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASE + 0x1400)
#define ADC1_BASEADDR					(APB2PERIPH_BASE + 0x2000)
#define SDIO_BASEADDR					(APB2PERIPH_BASE + 0x2C00)
#define SPI1_I2S1_BASEADDR				(APB2PERIPH_BASE + 0x3000)
#define SPI4_I2S4_BASEADDR				(APB2PERIPH_BASE + 0x3400)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR					(APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASEADDR					(APB2PERIPH_BASE + 0x4000)
#define TIM10_BASEADDR					(APB2PERIPH_BASE + 0x4400)
#define TIM11_BASEADDR					(APB2PERIPH_BASE + 0x4800)
#define SPI5_I2S5_BASEADDR				(APB2PERIPH_BASE + 0x5000)


/********************* PERIPHERAL REGISTER DEFINITION STRUCTURES *************************/
/*
 * NOTE: REGISTERS OF A PERIPHERAL ARE SPECIFIC TO MCU
 * e.g:  NUMBER OF REGISTERS OF SPI PERIPHERAL OF STM32F4x FAMILY OF MCUs MAY BE DIFFERENT(MORE OR LESS)
 * COMPARED TO NUMBER OF REGISTER OF SPI PERIPHERAL OF STM32Lx OR STM32F0x FAMILY OF MCUs
 */


/*  peripheral definition  structure GPIO register (6)*/
typedef struct
{
	__vo uint32_t MODER;		/*GPIO port mode register*/
	__vo uint32_t OTYPER;	/*GPIO port output type register*/
	__vo uint32_t OSPEEDR;	/*GPIO port output speed register*/
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register*/
	__vo uint32_t IDR;		/*GPIO port input data register*/
	__vo uint32_t ODR;		/*GPIO port output data register*/
	__vo uint32_t BSRR;		/*GPIO port bit set/reset register*/
	__vo uint32_t LCKR;		/*GPIO port configuration lock register*/
	__vo uint32_t AFR[2];	/*GPIO alternate function AFR[0]=low & AFR[1]=high register*/
}GPIO_RegDef_t;


/*  peripheral definition  structure RCC register (8)*/
typedef struct
{
	__vo uint32_t CR;			/*RCC clock control register (RCC_CR)       				Address offset: 0x00 */
	__vo uint32_t PLLCFGR ;		/*RCC PLL configuration register (RCC_PLLCFGR)  			Address offset: 0x04 */
	__vo uint32_t CFGR;			/*RCC clock configuration register (RCC_CFGR)				Address offset: 0x08 */
	__vo uint32_t CIR;			/*RCC clock interrupt register (RCC_CIR)					Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;		/*RCC AHB1 peripheral reset register (RCC_AHB1RSTR)			Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;		/*RCC AHB2 peripheral reset register (RCC_AHB2RSTR) 		Address offset: 0x14 */
		 uint32_t RESERVED0[2];	/*Address offset: 0x18 for [0] & 0x1C for [1]									 */
	__vo uint32_t APB1RSTR;		/*RCC APB1 peripheral reset register for (RCC_APB1RSTR)		Address offset: 0x20 */
	__vo uint32_t APB2RSTR;		/*RCC APB2 peripheral reset register (RCC_APB2RSTR)			Address offset: 0x24 */
		 uint32_t RESERVED1[2];	/*Address offset: 0x28 for [0] & 0x2C for [1]									 */
	__vo uint32_t AHB1ENR;		/*RCC AHB1 peripheral clock enable register (RCC_AHB1ENR) 	Address offset: 0x30 */
	__vo uint32_t AHB2ENR;		/*RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)	Address offset: 0x34 */
		 uint32_t RESERVED2[2];	/*Address offset: 0x38 for [0] & 0x3C for [1]									 */
	__vo uint32_t APB1ENR;		/*RCC APB1 peripheral clock enable register (RCC_APB1ENR)	Address offset: 0x40 */
	__vo uint32_t APB2ENR;		/*RCC APB2 peripheral clock enable register(RCC_APB2ENR)	Address offset: 0x44 */
	     uint32_t RESERVED3[2];	/*Address offset: 0x48 for [0] & 0x4C for [1]									 */
	__vo uint32_t AHB1LPENR;	/*RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR) Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;	/*RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR) Address offset: 0x54 */
		 uint32_t RESERVED4[2];	/*Address offset: 0x58 for [0] & 0x5C for [1]									 */
	__vo uint32_t APB1LPENR;	/*RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR) Address offset: 0x60 */
	__vo uint32_t APB2LPENR;	/*RCC APB2 peripheral clock enabled in low power mode register(RCC_APB2LPENR) Address offset: 0x64 */
		 uint32_t RESERVED5[2];	/*Address offset: 0x68 for [0] & 0x6C for [1]									 */
	__vo uint32_t BDCR;			/*RCC Backup domain control register (RCC_BDCR)				Address offset: 0x70  */
	__vo uint32_t CSR;			/*RCC clock control & status register (RCC_CSR)				Address offset: 0x74  */
		 uint32_t RESERVED6[2];	/*Address offset: 0x78 for [0] & 0x7C for [1]									 */
	__vo uint32_t SSCGR;		/*RCC spread spectrum clock generation register (RCC_SSCGR)	Address offset: 0x80  */
	__vo uint32_t PLLI2SCFGR;	/*RCC PLLI2S configuration register (RCC_PLLI2SCFGR)		Address offset: 0x84  */
	__vo uint32_t DCKCFGR;		/*RCC Dedicated Clocks Configuration Register (RCC_DCKCFGR)	Address offset: 0x8C  */
}RCC_RegDef_t;



/*
 * peripheral definition  structure EXTI (45)
 */
typedef struct
{
	__vo uint32_t IMR;				/* Interrupt mask register  			 Address offset: 0x00 */
	__vo uint32_t EMR;		 		/* Event mask register					 Address offset: 0x04 */
	__vo uint32_t RTSR;				/* Rising trigger selection register	 Address offset: 0x08 */
	__vo uint32_t FTSR;				/* Falling trigger selection register	 Address offset: 0x0C */
	__vo uint32_t SWIER;			/* Software interrupt event register	 Address offset: 0x10 */
	__vo uint32_t PR;				/* Pending register						 Address offset: 0x14 */
}EXTI_RegDef_t;




/*
 * peripheral definition  structure SYSCFG (46)
 */
typedef struct
{
	__vo uint32_t MEMRMP;			/* SYSCFG memory remap register							 Address offset: 0x00 */
	__vo uint32_t PMC;		 		/* SYSCFG peripheral mode configuration register		 Address offset: 0x04 */
	__vo uint32_t EXTICR[4];		/* SYSCFG external interrupt configuration register 1	 Address offset: 0x08 */
	__vo uint32_t RESERVED[2];		/* SYSCFG external interrupt configuration register 1	 Address offset: 0x18 */
	__vo uint32_t CMPCR;			/* Compensation cell control register 					 Address offset: 0x20 */
}SYSCFG_RegDef_t;








/*
 * peripheral definition  (peripheral addresses typecasted to xxx_RegDef_t (7))
 */
#define GPIOA 				((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC 				((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD 				((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE 				((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOH 				((GPIO_RegDef_t *)GPIOH_BASEADDR)



/*
 * peripheral definition for RCC (9)
 */
#define RCC					((RCC_RegDef_t *)RCC_BASEADDR)


/*
 * peripheral definition for interrupt (44)
 */
#define EXTI					((EXTI_RegDef_t *)EXTI_BASEADDR)



/*
 * peripheral definition for syscfg (47)
 */
#define SYSCFG					((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)






/*
 * Clock enable macro for GPIOx peripheral (10)
 */
#define	GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0) )
#define	GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1) )
#define	GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2) )
#define	GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3) )
#define	GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4) )
#define	GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7) )


/*
 * Clock enable macro for I2Cx peripheral (11)
 */
#define	I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21) )
#define	I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22) )
#define	I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23) )


/*
 * Clock enable macro for SPIx peripheral (12)
 */
#define	SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12) )
#define	SPI4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 13) )

#define	SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14) )
#define	SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15) )



/*
 * Clock enable macro for USARTx peripheral (13)
 */
#define	USART2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 17) )
#define	USART1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 4 ) )
#define	USART6_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 5 ) )


/*
 * Clock enable macro for SYSCFG peripheral (14)
 */
#define	SYSCFG_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 14) )





/*
 * Clock disable macro for GPIOx peripheral (15)
 */
#define	GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0) )
#define	GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1) )
#define	GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2) )
#define	GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3) )
#define	GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4) )
#define	GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7) )


/*
 * Clock disable macro for I2Cx peripheral (16)
 */
#define	I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21) )
#define	I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22) )
#define	I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23) )


/*
 * Clock disable macro for SPIx peripheral (17)
 */
#define	SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12) )
#define	SPI4_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 13) )

#define	SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14) )
#define	SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15) )



/*
 * Clock disable macro for USARTx peripheral (18)
 */
#define	USART2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 17) )
#define	USART1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 4 ) )
#define	USART6_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 5 ) )


/*
 * Clock disable macro for SYSCFG peripheral (19)
 */
#define	SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 14) )


/*
 *  macro to reset GPIOx peripherals (42)
 */
#define	GPIOA_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 0) );  ( RCC->AHB1RSTR &= ~( 1 << 0) );}while(0)
#define	GPIOB_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 1) );  ( RCC->AHB1RSTR &= ~( 1 << 1) );}while(0)
#define	GPIOC_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 2) );  ( RCC->AHB1RSTR &= ~( 1 << 2) );}while(0)
#define	GPIOD_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 3) );  ( RCC->AHB1RSTR &= ~( 1 << 3) );}while(0)
#define	GPIOE_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 4) );  ( RCC->AHB1RSTR &= ~( 1 << 4) );}while(0)
#define	GPIOH_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 7) );  ( RCC->AHB1RSTR &= ~( 1 << 7) );}while(0)





/*
 * This macro to return (0 to 7 ,except 5 6 ) for a given gpio base address (47)
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA)?0: \
										 (x == GPIOB)?1: \
										 (x == GPIOC)?2: \
									     (x == GPIOD)?3: \
									     (x == GPIOE)?4: \
									     (x == GPIOH)?7:0 )





/*
 * IRQ (interrupt request) Numbers of STM32F411xE MCU
 * Note: update these macros with valid value according to MCU
 * TODO: you may this list for other peripherals
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40


/*
 *  				          Macros for all the possible Priority level NVIC IRQ PRIORITY
*/
#define NVIC_IRQ_PRIORITY_0				0
#define NVIC_IRQ_PRIORITY_1				1
#define NVIC_IRQ_PRIORITY_2				2
#define NVIC_IRQ_PRIORITY_3				3
#define NVIC_IRQ_PRIORITY_4				4
#define NVIC_IRQ_PRIORITY_5				5
#define NVIC_IRQ_PRIORITY_6				6
#define NVIC_IRQ_PRIORITY_7				7
#define NVIC_IRQ_PRIORITY_8				8
#define NVIC_IRQ_PRIORITY_9				9
#define NVIC_IRQ_PRIORITY_10			10
#define NVIC_IRQ_PRIORITY_11			11
#define NVIC_IRQ_PRIORITY_12			12
#define NVIC_IRQ_PRIORITY_13			13
#define NVIC_IRQ_PRIORITY_14			14
#define NVIC_IRQ_PRIORITY_15			15
#define NVIC_IRQ_PRIORITY_16			16
#define NVIC_IRQ_PRIORITY_17			17
#define NVIC_IRQ_PRIORITY_18			18
#define NVIC_IRQ_PRIORITY_19			19
#define NVIC_IRQ_PRIORITY_23			23
#define NVIC_IRQ_PRIORITY_40			40






/*some generic macros*/

#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET 			RESET








#include"stm32f411re_gpio_driver.h"

#endif /* INC_STM32F411RE_H_ */
