
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
u16 MOTOR_TEST[4] = {999,999,999,999};
char moto_t[4] = {0};
void TIM2_IRQHandler(void)
{

  AHRS_Geteuler();

	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
}


void TIM5_IRQHandler(void)		    //2.5ms中断一次
{	
//	static uint8_t get = 0;
//	static uint8_t situation = 2;
	u8 i = 0;
	if(TIM5->SR & TIM_IT_Update)	{   
    TIM5->SR = ~TIM_FLAG_Update;//清除中断标志
	  //AHRS_Geteuler();
	  if(flag.plus){
		  for(i = 0; i<4;i++)
			{
				MOTOR_TEST[i] += 50;
			}
		   flag.plus = 0;
			 moto_PwmSet(MOTOR_TEST); 
			IntToStr(MOTOR_TEST[0], moto_t); 
			u3_printf(moto_t);
			u3_printf("   ");
		}
		if(flag.minu){
		  for(i = 0; i<4;i++)
			{
				MOTOR_TEST[i] -= 50;
			}
		   flag.minu = 0;
			 moto_PwmSet(MOTOR_TEST);	
		  IntToStr(MOTOR_TEST[0], moto_t); 
			u3_printf(moto_t);
			u3_printf("   ");			
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
