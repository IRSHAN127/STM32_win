/*
 * stm32f411re_gpio_driver.c
 *
 *  Created on: Dec 14, 2023
 *      Author: irshan
 */
#include "stm32f411re_gpio_driver.h"


/*
 * 	@fn					- GPIO Peripheral clock control (24)
 *
 * 	@brief				- This Function enables or disables peripheral clock for given GPIO port
 *
 * 	@param[1]			- Base address of the GPIO Peripheral
 * 	@param[2]			- ENABLE or DISABLE macros
 *	@param[3]			-
 *
 *
 *	@return				- none
 *
 *	@Note				- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/*
 * 	@fn					- Initialize GPIO (25)
 *
 * 	@brief				- This Function initialize GPIO
 *
 * 	@param[1]			- This holds the GPIO pin configuration setting
 * 	@param[2]			-
 *	@param[3]			-
 *
 *
 *	@return				- none
 *
 *	@Note				- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	/*(40)*/
	uint32_t temp=0;
	//1. Config the mode of the GPIO pin

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG)
		{
			temp=  pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~(0X3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

			pGPIOHandle->pGPIOx->MODER |= temp;
		}
		else
		{
			// interrupt config
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
			{

				//1. configure the FTSR

				EXTI->FTSR |= (1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

				// CLEARING THR RAISING CONFIG IF PREVIOUSLY SET

				EXTI->RTSR &= ~(1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			}
			else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
			{
				//1. configure the RTSR

				EXTI->RTSR |= (1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

				// CLEARING THR RAISING CONFIG IF PREVIOUSLY SET

				EXTI->FTSR &= ~(1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			}
			else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
			{
						//1. configure the FTSR and RTSR
				EXTI->RTSR |= (1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

				// CLEARING THR RAISING CONFIG IF PREVIOUSLY SET

				EXTI->FTSR |= (1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			}

			//2.Config the gpio port  selection in syscfg_EXTICR
			//temp1 for finding which EXTICRx, temp1 for finding from which bit of of EXTICR[x] we have to write
			uint8_t temp1 = (uint8_t)((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4);
			uint8_t temp2 = (uint8_t)((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4);

			uint8_t portcode =GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1]= portcode<< (temp2 *4);

			//3. Enable the EXTI interrupt delivery using IMR
			EXTI->IMR |= (1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		}

		//2. config the speed

		temp=0;
		temp= pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0X3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR |=temp;

		temp=0;

		//3. config the pupd settings

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0X3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR |=temp;

		temp=0;

		//4.config the optype
		temp =pGPIOHandle->GPIO_PinConfig.GPIO_PinOType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER&= ~(0X1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER |=temp;

		temp=0;


		//5.config the alt funcxfdn

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_ALTFUN)
		{
			//config the alt fun registers

			uint32_t temp1,temp2;

			temp1 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
			temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0XF << (4 * temp2));

			pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
		}

}




/*
 * 	@fn					- De-initialize GPIO (26)
 *
 * 	@brief				- This Function De-initialize GPIO
 *
 * 	@param[1]			- This holds the GPIO pin reset setting (rcc reset register)
 * 	@param[2]			-
 *	@param[3]			-
 *
 *
 *	@return				- none
 *
 *	@Note				- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	/*(41)*/

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}




/*
 * 	@fn					- Read From Input Pin (27)
 *
 * 	@brief				- This Function is used for reading data from any pin
 *
 * 	@param[1]			- This holds the GPIO port address
 * 	@param[2]			- This holds the GPIO pin Number
 *	@param[3]			-
 *
 *
 *	@return				- pin data that read 1 bit
 *
 *	@Note				- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
	/*43*/
	uint8_t value;
	value =(uint8_t )((pGPIOx->IDR >> PinNumber)&0X00000001);
	return value;

}




/*
 * 	@fn					- Read From Input port (28)
 *
 * 	@brief				- This Function is used for reading data from any port
 *
 * 	@param[1]			- This holds the GPIO port address
 * 	@param[2]			-
 *	@param[3]			-
 *
 *
 *	@return				- port data 16 bits
 *
 *	@Note				- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value =(uint16_t )pGPIOx->IDR;
	return value;
}




/*
 * 	@fn					- write to output pin (29)
 *
 * 	@brief				- This Function is used for writting data to a pin
 *
 * 	@param[1]			- This holds the GPIO port address
 * 	@param[2]			- This holds the Pin Number
 *	@param[3]			- This holds the Value to be written in pin 1 bit
 *
 *
 *	@return				- none
 *
 *	@Note				- none
 *
 */
void GPIO_WriteFromOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber , uint8_t value)
{

	if(value== GPIO_PIN_SET)
	{
		//write 1 on the pinn
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		//write 0 on the pinn
		pGPIOx->ODR &= ~(1<<PinNumber);

	}
}


/*
 * 	@fn					- write to output pin (30)
 *
 * 	@brief				- This Function is used for writting data to a port
 *
 * 	@param[1]			- This holds the GPIO port address
 * 	@param[2]			- This holds the Value to be written in port 16 bits
 *	@param[3]			-
 *
 *
 *	@return				- none
 *
 *	@Note				- none
 *
 */
void GPIO_WriteFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}






/*
 * 	@fn					- Toggle output pin (31)
 *
 * 	@brief				- This Function is used for toggling data of a pin
 *
 * 	@param[1]			- This holds the GPIO port address
 * 	@param[2]			- This holds Pin Number
 *	@param[3]			-
 *
 *
 *	@return				- none
 *
 *	@Note				- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber )
{
	pGPIOx->ODR ^= (1<<PinNumber);
}




/*
 * 	@fn					- GPIO IRQ Configuration (32)
 *
 * 	@brief				- This actually used to configure the IRQ number of the GPIO pin like enabling it, setting up the priority
 *
 * 	@param[1]			- This holds the IRQ Number
 * 	@param[2]			- This holds IRQPriority
 *	@param[3]			- ENABLE or DISABLE macros
 *
 *
 *	@return				- none
 *
 *	@Note				- Here API is actually the processor specific.
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnOrDi )
{
	if(EnOrDi == ENABLE)
		{
			if(IRQNumber <=31)
			{
				//program on ISER0 Register
				*NVIC_ISER0 |= (1<< IRQNumber);
			}
			else if(IRQNumber > 31  && IRQNumber <64)
			{
				//program on ISER1 Register
				*NVIC_ISER1 |= (1<< (IRQNumber%32) );
			}
			else if(IRQNumber >=64 && IRQNumber <96)
			{
				//program on ISER2 Register
				*NVIC_ISER2 |= (1<< (IRQNumber%64) );
			}
		}
		else
		{
					if(IRQNumber <=31)
					{
						//program on ICER0 Register
						*NVIC_ICER0 |= (1<< IRQNumber);
					}
					else if(IRQNumber > 31  && IRQNumber <64)
					{
						//program on ICER1 Register
						*NVIC_ICER1 |= (1<< (IRQNumber%32) );

					}
					else if(IRQNumber >=64 && IRQNumber <96)
					{
						//program on ICER2 Register
						*NVIC_ICER2 |= (1<< (IRQNumber%64) );

					}

		}
}



/*
 * 	@fn					- GPIO IRQ priority set (48)
 *
 * 	@brief				-
 *
 * 	@param[1]			- This holds the IRQ Priority Number
 * 	@param[2]			-
 *	@param[3]			-
 *
 *
 *	@return				- none
 *
 *	@Note				- none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the IPR register(49)
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTATION);
	*(NVIC_PR_BASE_ADDR + iprx  ) |= ( IRQPriority << shift_amount);
}


/*
 * 	@fn					- GPIO IRQ Configuration (33)
 *
 * 	@brief				- whenever interrupt occurs so they user application can call this function in order to process that interrupt.
 *
 * 	@param[1]			- This holds the Pin Number
 * 	@param[2]			-
 *	@param[3]			-
 *
 *
 *	@return				- none
 *
 *	@Note				- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR Register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber ))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber );
	}
}





