
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

	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
}


void TIM5_IRQHandler(void)		    //2.5ms中断一次
{	
	static uint8_t get = 0;
	static uint8_t situation = 2;
	if(TIM5->SR & TIM_IT_Update)	{    
    TIM5->SR = ~TIM_FLAG_Update;//清除中断标志	
    time++;
		//AHRS_Geteuler();	
    //蓝牙模块的
		if(time>40){
		USART3_RX_STA |= 1<<15;					//强制标记接收完成
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
 	  if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)   //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 1)
			{
				TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				Rise=TIM_GetCapture4(TIM3);
      }
			else
			{
				TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
				Drop=TIM_GetCapture4(TIM3);
 				if(Rise>Drop)  US100_Alt_Temp = 65535-Rise + Drop;
				else 	             US100_Alt_Temp = Drop - Rise;
			
				if(US100_Alt_Temp>20000)   US100_Alt_Temp=Alt_Last; 
				else    					Alt_Last=US100_Alt_Temp;

				US100_Alt=US100_Alt_Temp*34/200;//这里只是获得高度，不做处理了,优化时可以加入倾角补偿
				
				ultra_start_f=1; 

				//以下是防止异常的情况，及时保护，因为超出超声波测距范围后，数据会突然变成很小，比如11.084，并基本保持不动
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
