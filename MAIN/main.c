/********************************************************
模块时钟说明：
主频： 72MHz
APB1： 分频2 36MHz
TIM2 - TIM7： 72MHz
	
//static uint8_t i = 0;
//	if(timeflag==1){
//			time++;
//		}
//		//蓝牙模块的
//		if(time>40){
//		USART3_RX_STA |= 1<<15;					//强制标记接收完成
//		timeflag = 0;
//		time = 0;
////		get = Str_Equal( AltraHighJudge , USART3_RX_BUF,4);
////		if(get) situation = 0;
////		get = Str_Equal("stop", USART3_RX_BUF,4);
////		if(get) situation = 1;
////		get = Str_Equal("reset", USART3_RX_BUF,5);
////		if(get) situation = 3;
////		
////    switch(situation)
////		{
////		  case 0: 			
////			flag.FlightMode  = ULTRASONIC_High;
////      flag.ARMED  = 1; break;
////			case 1: 
////			flag.ARMED = 0; break;
////			case 3:
////			NVIC_SystemReset(); break;			
////			default: break;
////		}
//    		
//		u3_printf(USART3_RX_BUF);
//		array_assignu8(USART3_RX_BUF,0x00,USART3_MAX_RECV_LEN);
//		USART3_RX_STA = 0;
//}
////   if(flag.calicomplete == 1){
//	 Dis_Float(0, 1, IMU.Pitch,4) ;//Display signed int
//	 Dis_Float(0, 2, IMU.Roll,4) ;//Display signed int
//   Dis_Float(0, 3, IMU.Yaw,4) ;//Display signed int
////   }

//static uint8_t get = 0;
//static uint8_t situation = 2;
////static uint8_t startcnt = 0;
////static uint8_t begin = 0;			 


//  //发送超声波脉冲
//	i++;
//  if(i==50)
//	{
//	  i = 0;
//    Ultrasonic_Pulsing();
//		//Dis_int(22,2,US100_Alt,4);
//	}
**********************************************************/



#include "mpu6050.h"
#include "iic.h"
#include "include.h"
#include "LED.h"
#include "OLED.h"
#include "hmc5883.h"
#include "timer.h"
#include "motor.h"
#include "Ultrasonic.h"
#include "hc05.h"
#include "control.h"
#include "ultracontrol.h"
#include "ahrs.h"
#include "stmflash.h"
#include "hc05.h"
Flag_t flag;
void Init(void);
void Test(void);
void	paramLoad(void);
void Bootloader_Set(void);

int main(void)
{
  Init();
	while(1)
	{
   if(flag.calicomplete){
	 Dis_Float(0, 1, IMU.Pitch,2) ;//Display signed int
	 Dis_Float(0, 2, IMU.Roll,2) ;//Display signed int
   Dis_Float(0, 3, IMU.Yaw+180,2) ;//Display signed int 
	 Dis_int(0,4,flag.MagIssue,1);
	 }
	}
}



void Init(void)
{
	
	//Bootloader_Set();
		
	delay_init(72);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
  Led_Init() ;
	
  OLED_Init();
	
	usart3_init(115200);
	
	flag.FlightMode = MANUAL_High;
	
	I2C_INIT();
  delay_ms(800);
	
	
	flag.MpuExist = MPU6050_Init();
  flag.MagExist = Init_HMC5883L();

	/***************Motor init***************/
	TIM4_PWM_Init(19999,71);
	//moto_Cali();
	moto_STOP(); 
	/************ Ultrasonic Init***********/
	Ultrasonic_Config();
	PWM_IN_Config();
	
	MPU6050_Cali();
	paramLoad();
	

	TIM5_Config();

	EnTIMER;
}

void Test(void)
{
  Fuse_RegValue();
  OLED_P6x8Str(0,2,"A-X");//小字符串
	Dis_int(22, 2, (int)MPU_Data.Acce.origin.x,4); 
	OLED_P6x8Str(0,3,"A-Y");//小字符串
	Dis_int(22, 3, (int)MPU_Data.Acce.origin.y,4);
	OLED_P6x8Str(0,4,"A-Z");//小字符串
	Dis_int(22, 4, (int)MPU_Data.Acce.origin.z,4);
	
	HMC5883lRead(hmcdata);
	OLED_P6x8Str(64,2,"H-X");//小字符串
	Dis_int(86, 2, (int)MPU_Data.Acce.origin.x,4); 
	OLED_P6x8Str(64,3,"H-Y");//小字符串
	Dis_int(86, 3, (int)MPU_Data.Acce.origin.y,4);
	OLED_P6x8Str(64,4,"H-Z");//小字符串
	Dis_int(86, 4, (int)MPU_Data.Acce.origin.z,4);
	
	delay_ms(500);
}

void	paramLoad(void)
{
	//The data of pitch
	ctrl.pitch.shell.kp = 0;//4;
	ctrl.pitch.shell.ki = 0;//0.02;
	
	ctrl.pitch.core.kp = 0.5;//1.4;  
	ctrl.pitch.core.ki = 0;//0.45; 
	ctrl.pitch.core.kd = 0;//0.70;
	
	//The data of roll
	ctrl.roll.shell.kp = 4;
	ctrl.roll.shell.ki = 0.02;

	ctrl.roll.core.kp = 1.4;
	ctrl.roll.core.ki = 0.45;
	ctrl.roll.core.kd = 0.70;
	
	//The data of yaw
	ctrl.yaw.shell.kp = 5;
	ctrl.yaw.shell.kd = 0;
	
	ctrl.yaw.core.kp = 1.8;
	ctrl.yaw.core.ki = 0;
	ctrl.yaw.core.kd = 0.1;
	
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 20;
	ctrl.roll.shell.increment_max = 20;
	
	ctrl.ctrlRate = 0;
	
	
	WZ_Speed_PID_Init();    //高度控制PID初始化
	Ultra_PID_Init();       //超声波PID
}


void Bootloader_Set(void)
{
 	u16 i;
	
	// 设置偏移量 
	SCB->VTOR = FLASH_BASE | FLASH_EXCURSION ; 
	
	i=0x0505;
	STMFLASH_Write(pro_FALG_ADD,&i,1);   
}
