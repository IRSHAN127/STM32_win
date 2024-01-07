/*
 * stm32f411re_gpio_driver.h
 *
 *  Created on: Dec 14, 2023
 *      Author: irsha
 */

#ifndef INC_STM32F411RE_GPIO_DRIVER_H_
#define INC_STM32F411RE_GPIO_DRIVER_H_

#include"stm32f411re.h"


/* gpio configuration (34)*/
typedef struct
{
	uint8_t GPIO_PinNumber;					/*Possible Value from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;					/*Possible Value from @GPIO_PIN_MODE*/
	uint8_t GPIO_PinSpeed;					/*Possible Value from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;			/*Possible Value from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOType;					/*Possible Value from @GPIO_PIN_OPTYPE*/
	uint8_t GPIO_PinAltFunMode;				/*Possible Value from @GPIO_PIN_ALT_FUN*/

}GPIO_PinConfig_t;




/*
 * This is handle structure for a GPIO pin (20.0)
 */


typedef struct
{
	//pointer to hold base address of the GPIO peripheral

	GPIO_RegDef_t *pGPIOx; 				/*This hold the base address of GPIO port to which the pin belongs ,we can define it as GPIOA ,B ..etc*/
	GPIO_PinConfig_t GPIO_PinConfig; 	/*This holds the GPIO pin configuration setting */

}GPIO_Handle_t;



/*
 * @GPIO_PIN_NUMBER
 * GPIO Pin Numbers (35)
 */

#define GPIO_PIN_NO_0 	0
#define GPIO_PIN_NO_1 	1
#define GPIO_PIN_NO_2 	2
#define GPIO_PIN_NO_3 	3
#define GPIO_PIN_NO_4 	4
#define GPIO_PIN_NO_5 	5
#define GPIO_PIN_NO_6 	6
#define GPIO_PIN_NO_7 	7
#define GPIO_PIN_NO_8 	8
#define GPIO_PIN_NO_9 	9
#define GPIO_PIN_NO_10 	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12 	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14 	14
#define GPIO_PIN_NO_15	15




/*
 * @GPIO_PIN_MODE
 * GPIO pin possibel macros (36)
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFUN	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4 /* INPUT FALLING EDGE*/
#define GPIO_MODE_IT_RT		5 /* INPUT RAISING EDGE*/
#define GPIO_MODE_IT_RFT	6 /* INPUT RAISING & FALLING EDGE*/




/*
 * @GPIO_PIN_OPTYPE
 * GPIO Possible output types (37)
 */
#define GPIO_OP_TYPE_PP      0
#define GPIO_OP_TYPE_OD      1



/*
 * @GPIO_PIN_SPEED
 * GPIO Possible output Speeds (38)
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/*
 * @GPIO_PIN_PUPD
 * GPIO Possible pull up and pull down config macros (39)
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

















/**********************************************************************************************
 *                             APIs supported by this driver
 * 				for more information about API check the function definitios
 *********************************************************************************************/

/*
 * 	GPIO Peripheral clock control (20.1)
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi);


/*
 * Init and De-init (21)
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write (22)
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteFromOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber , uint8_t value);
void GPIO_WriteFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber );

/*
 * IRQ configuration and ISR Handling (23)
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnOrDi );	/*This actually used to configure the IRQ number of the GPIO pin like enabling it, setting up the priority*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);	/*whenever interrupt occurs so they user application can call this function in order to process that interrupt.*/










#endif /* INC_STM32F411RE_GPIO_DRIVER_H_ */
