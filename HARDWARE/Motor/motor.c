

#include "motor.h"


//TIM4 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
//���Ҫ�Ĳ������ӵ�һ�иĵ����һ��
void TIM4_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4ʱ��ʹ��    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	//ʹ��PORTA,Bʱ��	
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;           //GPIOA6,GPIOA7,GPIOB0,GPIOB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //��ʼ��B6, B7, B8, B9

	
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
	
	//��ʼ��TIM4 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC1
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC2
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC3
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC4
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR4�ϵ�Ԥװ�ؼĴ���
  
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPEʹ��
	
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
	
}  



void moto_PwmRflash(u16 *Moter)
{		
	static u8 i = 0;
	for(i=0;i<MOTOR_NUM;i++)
	{
     if(*(Moter+i) > Moto_PwmMax)  *(Moter+i) = Moto_PwmMax;
  }
	for(i=0;i<MOTOR_NUM;i++)
	{
     if(*(Moter+i) <= 0 )  *(Moter+i) = 0;
  }
  TIM_SetCompare1(TIM4,(999 + *(Moter++)));
	TIM_SetCompare2(TIM4,(999 + *(Moter++)));
	TIM_SetCompare3(TIM4,(999 + *(Moter++)));
	TIM_SetCompare4(TIM4,(999 + *(Moter)));
}

void moto_STOP(void)
{
  TIM_SetCompare1(TIM4,999);
	TIM_SetCompare2(TIM4,999);
	TIM_SetCompare3(TIM4,999);
	TIM_SetCompare4(TIM4,999);
}

void moto_Cali(void)
{
	//���
  TIM_SetCompare1(TIM4,1999);
	TIM_SetCompare2(TIM4,1999);
	TIM_SetCompare3(TIM4,1999);
	TIM_SetCompare4(TIM4,1999);
	delay_ms(1000);
	TIM_SetCompare1(TIM4,999);
	TIM_SetCompare2(TIM4,999);
	TIM_SetCompare3(TIM4,999);
	TIM_SetCompare4(TIM4,999);
	delay_ms(100);
}
