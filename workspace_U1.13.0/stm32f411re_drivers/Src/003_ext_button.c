/*
 * 001_led_toggle.c
 *
 *  Created on: Dec 15, 2023
 *      Author: irshan
 */

#include"stm32f411re.h"
#define HIGH 				1
#define LOW 				0
#define BTN_PRESSED 		LOW


void delay(uint32_t Delay)
{
	for(uint32_t  i = 0 ; i < Delay ; i ++);
}


int main(void)
{
	GPIO_Handle_t GpioLed ,GpioBtn;

	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx=GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED )
		{
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
			delay(500000);
		}


		/*if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED )
		{
			GPIO_WriteFromOutputPin(GPIOA, GPIO_PIN_NO_5,GPIO_PIN_SET);
			//			delay(500000);
		}
		else
		{
			GPIO_WriteFromOutputPin(GPIOA, GPIO_PIN_NO_5,GPIO_PIN_RESET);
		}*/


	}

	return 0;
}
