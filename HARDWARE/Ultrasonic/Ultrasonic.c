
#include "Ultrasonic.h"

#define	ECHO_PORT      GPIOB		  //������ģ��ECHO�����ź�����˿�---�ɿ�CH7����PWM 
#define	ECHO_PIN       GPIO_Pin_0	//ECHO--CH7��PB0�� 

#define	TRIG_PORT      GPIOB		  //������ģ��TRIG�����ź�����˿�---�ɿ�CH8���PWM
#define	TRIG_PIN       GPIO_Pin_1 //TRIG--CH8��PB1��
#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  4
#define Ultrasonic_MAX_Height 2500   //�����������Ч�߶ȣ���λ��mm


float US100_Alt;
float US100_Alt_delta;
float US100_Alt_Last=0;

u8 Ultrasonic_OK;
s8 ultra_start_f;
u16 Drop,Rise;
vu16 US100_Alt_Temp=0,Alt_Last=0; 


/*******************************************
�������ز���⡣�����ߵ�ƽʱ��õ�����
********************************************/
void PWM_IN_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM1_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the TIM1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); //TIM1 ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ�� PORTB ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;					    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // ����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        
	GPIO_Init(GPIOB, &GPIO_InitStructure);	        //��ʼ������GPIO
	
	TIM_TimeBaseStructure.TIM_Prescaler=83; //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=0xffff; //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision  =  TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2; //ѡ������� IC1 ӳ�䵽 TI1 ��
	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�����ز���
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽 TI1 ��
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //���������Ƶ,����Ƶ
	TIM1_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM1_ICInitStructure); //��ʼ�� TIM1 ���벶�����
	
	TIM_ITConfig(TIM1,TIM_IT_CC2,ENABLE);//�������ж�
	TIM_Cmd(TIM1,ENABLE ); //ʹ�ܶ�ʱ�� 5
}



/**************************************************************************
������TRIG�ź�����ʹ�ܡ�ʹ�ô����������������������ź�
***************************************************************************/
void Ultrasonic_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	       
  /* config the extiline(PB0) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = TRIG_PIN;					    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    // �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        
	GPIO_Init(TRIG_PORT, &GPIO_InitStructure);	        //��ʼ������GPIO 	
	GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}


void Ultrasonic_Pulsing(void)
{
  GPIO_SetBits(TRIG_PORT,TRIG_PIN);		  //��>10US�ĸߵ�ƽ
	delay_us(20);
  GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}





