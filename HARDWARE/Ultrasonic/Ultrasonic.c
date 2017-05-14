
#include "Ultrasonic.h"

#define	ECHO_PORT      GPIOB		  //超声波模块ECHO回响信号输出端口---飞控CH7输入PWM 
#define	ECHO_PIN       GPIO_Pin_1	//ECHO--CH7（PB0） 

#define	TRIG_PORT      GPIOB		  //超声波模块TRIG触发信号输入端口---飞控CH8输出PWM
#define	TRIG_PIN       GPIO_Pin_0 //TRIG--CH8（PB1）
#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  4
#define Ultrasonic_MAX_Height 2500   //超声波最高有效高度，单位是mm


float US100_Alt;
float US100_Alt_delta;
float US100_Alt_Last=0;

u8 Ultrasonic_OK;
s8 ultra_start_f;
u16 Drop,Rise;
vu16 US100_Alt_Temp=0,Alt_Last=0; 


/*******************************************
超声波回波检测。测量高电平时间得到距离
********************************************/
void PWM_IN_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM3_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the TIM3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //TIM3 时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能 PORTB 时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;					    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        
	GPIO_Init(GPIOB, &GPIO_InitStructure);	        //初始化外设GPIO
	
	TIM_TimeBaseStructure.TIM_Prescaler=71; //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=0xffff; //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision  =  TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //选择输入端 IC1 映射到 TI1 上
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到 TI1 上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //配置输入分频,不分频
	TIM3_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure); //初始化 TIM3 输入捕获参数
	
	TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);//允许捕获中断
	TIM_Cmd(TIM3,ENABLE ); //使能定时器 5
}



/**************************************************************************
超声波TRIG信号引脚使能。使用此引脚来触发超声波发送信号
***************************************************************************/
void Ultrasonic_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	       
  /* config the extiline(PB0) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = TRIG_PIN;					    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    // 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        
	GPIO_Init(TRIG_PORT, &GPIO_InitStructure);	        //初始化外设GPIO 	
	GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}


void Ultrasonic_Pulsing(void)
{
  GPIO_SetBits(TRIG_PORT,TRIG_PIN);		  //送>10US的高电平
	delay_us(20);
  GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}





