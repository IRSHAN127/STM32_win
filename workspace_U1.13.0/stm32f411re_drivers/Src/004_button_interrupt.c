/*

* 001_led_toggle.c

*

* Created on: Dec 15, 2023

* Author: irshan

*/



#include"stm32f411re.h"

#include<string.h>

#define HIGH 1

#define LOW 0

#define BTN_PRESSED LOW





void delay(uint32_t Delay)

{

for(uint32_t i = 0 ; i < Delay ; i ++);

}





int main(void)

{

GPIO_Handle_t GpioLed ,GpioBtn;

memset(&GpioLed,0,sizeof(GpioLed));

memset(&GpioBtn,0,sizeof(GpioBtn));



GpioLed.pGPIOx=GPIOA;

GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;

GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;

GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;

GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



GPIO_PeriClockControl(GPIOA, ENABLE);



GPIO_Init(&GpioLed);



GpioBtn.pGPIOx=GPIOD;

GpioBtn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_2;

GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;

GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;



GPIO_PeriClockControl(GPIOD, ENABLE);



GPIO_Init(&GpioBtn);





//IRQ Configuration

GPIO_IRQPriorityConfig(IRQ_NO_EXTI2, NVIC_IRQ_PRIORITY_15);

GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, ENABLE);



while(1);



return 0;

}





void EXTI2_IRQHandler(void)

{

delay(2000000);

GPIO_IRQHandling(GPIO_PIN_NO_2);



GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);



}
