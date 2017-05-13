#ifndef 	_LED_
#define   _LED_
#include "sys.h"
#define GPIO_LED_RGB	     	GPIOD     //三色的LED对应PD
#define GPIO_LED_BLUE       GPIOE			//单色的LED对应PE
#define LED_PIN_R         	GPIO_Pin_0//三色的LED_R对应PD0
#define LED_PIN_G          	GPIO_Pin_1//三色的LED_G对应PD1
#define LED_PIN_B          	GPIO_Pin_3//三色的LED_B对应PD3

#define LED_PIN_BLUE1       GPIO_Pin_10//单色LED1对应PE10
#define LED_PIN_BLUE2       GPIO_Pin_12//单色LED1对应PE12

#define LED_R  							PDout(0)//实现PD0位带操作
#define LED_G  							PDout(1)//实现PD1位带操作
#define LED_B  							PDout(3)//实现PD3位带操作

#define LED_BLUE1          	PEout(10)//实现PE10位带操作
#define LED_BLUE2          	PEout(12)//实现PE12位带操作

void Led_Init(void);//初始化LED

#endif
