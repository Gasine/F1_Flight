#include "LED.h"

/***********************************************************************************
															���ߣ�HENTO TEAM
���ܣ�ʵ�ַɿ�����LED�ĳ�ʼ��
��ע��LED��Ӧ�ܽ����£�
��ɫLED1��PE10  ��ɫLED2��PE12
��ɫLED_R:PD0  ��ɫLED_G:PD1  ��ɫLED_B:PD3���led.h�ļ�
***********************************************************************************/
void Led_Init(void)
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����GPIOD&GPIOE������ʱ��*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE); 

	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
	GPIO_InitStructure.GPIO_Pin = LED_PIN_R | LED_PIN_G | LED_PIN_B;	
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	/*������������Ϊ50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	/*���ÿ⺯������ʼ��GPIOD*/
	GPIO_Init( GPIO_LED_RGB, &GPIO_InitStructure);		  
	/* �ر�����led��	*/
	GPIO_SetBits( GPIO_LED_RGB, LED_PIN_R | LED_PIN_G | LED_PIN_B);	
	
	GPIO_InitStructure.GPIO_Pin = LED_PIN_BLUE1 | LED_PIN_BLUE2;	//��Ӱ��ϵ�LED��
	GPIO_Init( GPIO_LED_BLUE, &GPIO_InitStructure);	
	GPIO_SetBits(GPIO_LED_BLUE, LED_PIN_BLUE1 | LED_PIN_BLUE2);	
}
