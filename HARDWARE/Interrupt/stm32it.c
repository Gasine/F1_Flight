
#include "sys.h"
//extern EulerAngle IMU;
extern void Test(void);
void TIM2_IRQHandler(void)
{
//static uint8_t i = 0;
//static struct _target target = {0.0,0.0,0.0};
//static uint8_t get = 0;
//static uint8_t situation = 2;
////static uint8_t startcnt = 0;
////static uint8_t begin = 0;
//				 

//if(timeflag==1){
//			time++;
//		}
//		//蓝牙模块的
//		if(time>40){
//		USART3_RX_STA |= 1<<15;					//强制标记接收完成
//		timeflag = 0;
//		time = 0;
//		get = Str_Equal( AltraHighJudge , USART3_RX_BUF,4);
//		if(get) situation = 0;
//		get = Str_Equal("stop", USART3_RX_BUF,4);
//		if(get) situation = 1;
//		get = Str_Equal("reset", USART3_RX_BUF,5);
//		if(get) situation = 3;
//		
//    switch(situation)
//		{
//		  case 0: 			
//			flag.FlightMode  = ULTRASONIC_High;
//      flag.ARMED  = 1; break;
//			case 1: 
//			flag.ARMED = 0; break;
//			case 3:
//			NVIC_SystemReset(); break;			
//			default: break;
//		}
//    		
//		u3_printf(USART3_RX_BUF);
//		array_assignu8(USART3_RX_BUF,0x00,USART3_MAX_RECV_LEN);
//		USART3_RX_STA = 0;
//}

//	 Dis_Float(0, 1, IMU.Pitch,4) ;//Display signed int
//	 Dis_Float(0, 2, IMU.Roll,4) ;//Display signed int
//   Dis_Float(0, 3, IMU.Yaw,4) ;//Display signed int

//  //发送超声波脉冲
//	i++;
//  if(i==50)
//	{
//	  i = 0;
//    Ultrasonic_Pulsing();
//		Dis_unint(0,0,ultra_dis_lpf,4);
//	}
//  AHRS_Geteuler();
//	//MAG3110_DATA_READ();
//	//收集静止时的姿态角
////	while(startcnt<50)
////	{
////		begin = 1;
////		target.Pitch = IMU.Pitch;
////		target.Roll = IMU.Roll;
////		startcnt++;
////	}
////	if(begin)
//	CONTROL(target);
  Test();
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
}


//void TIM1_CC_IRQHandler(void)
//{    		
// 	  if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)   //捕获1发生捕获事件
//		{	
//			TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //清除中断标志位
//			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1)
//			{
//				TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//				Rise=TIM_GetCapture2(TIM1);
//      }
//			else
//			{
//				TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//				Drop=TIM_GetCapture2(TIM1);
// 				if(Rise>Drop)  US100_Alt_Temp = 65535-Rise + Drop;
//				else 	             US100_Alt_Temp = Drop - Rise;
//			
//				if(US100_Alt_Temp>20000)   US100_Alt_Temp=Alt_Last; 
//				else    					Alt_Last=US100_Alt_Temp;

//				US100_Alt=US100_Alt_Temp*34/200;//这里只是获得高度，不做处理了,优化时可以加入倾角补偿
//				
//				ultra_start_f=1; 

//				//以下是防止异常的情况，及时保护，因为超出超声波测距范围后，数据会突然变成很小，比如11.084，并基本保持不动
//				if(abs(US100_Alt-US100_Alt_Last)>=500){
//					US100_Alt=US100_Alt_Last;
//					Ultrasonic_OK=0;
//				}
//				else Ultrasonic_OK=1;
//				US100_Alt_delta=US100_Alt - US100_Alt_Last;
//				US100_Alt_Last=US100_Alt; 
//     }	
//		}
//}
