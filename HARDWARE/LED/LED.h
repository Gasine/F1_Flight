#ifndef 	_LED_
#define   _LED_
#include "sys.h"
#define GPIO_LED_RGB	     	GPIOD     //��ɫ��LED��ӦPD
#define GPIO_LED_BLUE       GPIOE			//��ɫ��LED��ӦPE
#define LED_PIN_R         	GPIO_Pin_0//��ɫ��LED_R��ӦPD0
#define LED_PIN_G          	GPIO_Pin_1//��ɫ��LED_G��ӦPD1
#define LED_PIN_B          	GPIO_Pin_3//��ɫ��LED_B��ӦPD3

#define LED_PIN_BLUE1       GPIO_Pin_10//��ɫLED1��ӦPE10
#define LED_PIN_BLUE2       GPIO_Pin_12//��ɫLED1��ӦPE12

#define LED_R  							PDout(0)//ʵ��PD0λ������
#define LED_G  							PDout(1)//ʵ��PD1λ������
#define LED_B  							PDout(3)//ʵ��PD3λ������

#define LED_BLUE1          	PEout(10)//ʵ��PE10λ������
#define LED_BLUE2          	PEout(12)//ʵ��PE12λ������

void Led_Init(void);//��ʼ��LED

#endif
