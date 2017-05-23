
#include "sys.h"
#include "Ultrasonic.h"
#include "OLED.h"
#include "Algorithm_math.h"
#include "hc05.h"
#include "control.h"
#include "ultracontrol.h"
#include "LED.h"
#include "Algorithm_quaternion.h"
#include "hc05.h"
extern EulerAngle IMU;
extern void Test(void);
void TIM2_IRQHandler(void)
{

  AHRS_Geteuler();

	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
}


void TIM5_IRQHandler(void)		    //2.5ms�ж�һ��
{	
	static uint8_t get = 0;
	static uint8_t situation = 2;
	if(TIM5->SR & TIM_IT_Update)	{    
    TIM5->SR = ~TIM_FLAG_Update;//����жϱ�־	
    time++;
		//AHRS_Geteuler();	
    //����ģ���
		if(time>40){
		USART3_RX_STA |= 1<<15;					//ǿ�Ʊ�ǽ������
		timeflag = 0;
		time = 0;
		get = Str_Equal( "high" , USART3_RX_BUF,4);
		if(get) situation = 0;
		get = Str_Equal("stop", USART3_RX_BUF,4);
		if(get) situation = 1;
		get = Str_Equal("reset", USART3_RX_BUF,5);
		if(get) situation = 3;
		
    switch(situation)
		{
		  case 0: 			
			flag.FlightMode  = ULTRASONIC_High;
      flag.ARMED  = 1; break;
			case 1: 
			flag.ARMED = 0; break;
			case 3:
			NVIC_SystemReset(); break;			
			default: break;
		}
    		
		u3_printf(USART3_RX_BUF);
		array_assignu8(USART3_RX_BUF,0x00,USART3_MAX_RECV_LEN);
		USART3_RX_STA = 0;
     }
	}
}

void TIM3_IRQHandler(void)
{    		
 	  if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)   //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); //����жϱ�־λ
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 1)
			{
				TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				Rise=TIM_GetCapture4(TIM3);
      }
			else
			{
				TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
				Drop=TIM_GetCapture4(TIM3);
 				if(Rise>Drop)  US100_Alt_Temp = 65535-Rise + Drop;
				else 	             US100_Alt_Temp = Drop - Rise;
			
				if(US100_Alt_Temp>20000)   US100_Alt_Temp=Alt_Last; 
				else    					Alt_Last=US100_Alt_Temp;

				US100_Alt=US100_Alt_Temp*34/200;//����ֻ�ǻ�ø߶ȣ�����������,�Ż�ʱ���Լ�����ǲ���
				
				ultra_start_f=1; 

				//�����Ƿ�ֹ�쳣���������ʱ��������Ϊ������������෶Χ�����ݻ�ͻȻ��ɺ�С������11.084�����������ֲ���
				if(abs(US100_Alt-US100_Alt_Last)>=500){
					US100_Alt=US100_Alt_Last;
					Ultrasonic_OK=0;
				}
				else Ultrasonic_OK=1;
				US100_Alt_delta=US100_Alt - US100_Alt_Last;
				US100_Alt_Last=US100_Alt; 
     }	
		}
}
