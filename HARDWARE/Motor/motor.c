

#include "motor.h"


//TIM4 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
//如果要改参数。从第一行改到最后一行
void TIM4_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	//使能PORTA,B时钟	
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;           //GPIOA6,GPIOA7,GPIOB0,GPIOB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化B6, B7, B8, B9

	
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器3
	
	//初始化TIM4 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC3
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC4
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR4上的预装载寄存器
  
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
	
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
	//最大
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
