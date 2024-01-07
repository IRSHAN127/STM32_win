/*
 * 002__led_toggle_opendrain.c
 *
 *  Created on: Dec 15, 2023
 *      Author: irshan
 */


#include"stm32f411re.h"


void delay(uint32_t Delay)
{
	for(uint32_t  i = 0 ; i < Delay ; i ++);
}


int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_OD;
	/*OPEN DRAIN WITH NO PULL UP & PULL DOWN  LED WONT BLINK*/
	//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	/*OPEN DRAIN WITH INTERNAL PULL UP LED WILL BLINK BUT LOW INTENSITY BECAUSE OF HIGH RESISTER(40K OHM) IN PULL UP*/
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	/*
	 * NOTE:
	 * TO BLINK NORAMALLY IN OPEN DRAIN , KEEP IN "NO PU PD" AND add EXTERNAL PULL UP BY CONNECTING 5V TO PA5 THROUGH 470 OHM RESISTER    */

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay(500000);//software delay
	}

	return 0;
}

