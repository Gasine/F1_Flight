
#include "sys.h"
#include "Ultrasonic.h"
#include "OLED.h"
#include "Algorithm_math.h"
#include "hc05.h"
#include "control.h"
#include "ultracontrol.h"
#include "LED.h"
#include "Algorithm_quaternion.h"
extern EulerAngle IMU;
extern void Test(void);
void TIM2_IRQHandler(void)
{

  AHRS_Geteuler();

	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
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
