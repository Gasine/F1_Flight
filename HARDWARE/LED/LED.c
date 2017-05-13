#include "LED.h"

/***********************************************************************************
															作者：HENTO TEAM
功能：实现飞控外设LED的初始化
备注：LED对应管脚如下：
单色LED1：PE10  单色LED2：PE12
三色LED_R:PD0  三色LED_G:PD1  三色LED_B:PD3详见led.h文件
***********************************************************************************/
void Led_Init(void)
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启GPIOD&GPIOE的外设时钟*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE); 

	/*选择要控制的GPIOC引脚*/															   
	GPIO_InitStructure.GPIO_Pin = LED_PIN_R | LED_PIN_G | LED_PIN_B;	
	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	/*设置引脚速率为50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	/*调用库函数，初始化GPIOD*/
	GPIO_Init( GPIO_LED_RGB, &GPIO_InitStructure);		  
	/* 关闭所有led灯	*/
	GPIO_SetBits( GPIO_LED_RGB, LED_PIN_R | LED_PIN_G | LED_PIN_B);	
	
	GPIO_InitStructure.GPIO_Pin = LED_PIN_BLUE1 | LED_PIN_BLUE2;	//添加板上的LED灯
	GPIO_Init( GPIO_LED_BLUE, &GPIO_InitStructure);	
	GPIO_SetBits(GPIO_LED_BLUE, LED_PIN_BLUE1 | LED_PIN_BLUE2);	
}
