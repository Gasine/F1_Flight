/********************************************************
Ä£¿éÊ±ÖÓËµÃ÷£º
Ö÷Æµ£º 72MHz
APB1£º ·ÖÆµ2 36MHz
TIM2 - TIM7£º 72MHz
**********************************************************/



#include "mpu6050.h"
#include "iic.h"
#include "include.h"
#include "LED.h"
#include "OLED.h"
#include "hmc5883.h"
#include "timer.h"
#include "motor.h"

Flag_t flag;
void Init(void);
void Test(void);

void Init(void)
{
  Led_Init() ;
  OLED_Init();
  I2C_INIT();
  delay_ms(200);
	MPU6050_Init();
	Init_HMC5883L();
	TIM4_PWM_Init(19999,71);
	moto_Cali();
	moto_STOP();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM2_Init(1799,99);

}

void Test(void)
{
  Fuse_RegValue();
  OLED_P6x8Str(0,2,"A-X");//Ð¡×Ö·û´®
	Dis_int(22, 2, (int)MPU_Data.Acce.origin.x,4); 
	OLED_P6x8Str(0,3,"A-Y");//Ð¡×Ö·û´®
	Dis_int(22, 3, (int)MPU_Data.Acce.origin.y,4);
	OLED_P6x8Str(0,4,"A-Z");//Ð¡×Ö·û´®
	Dis_int(22, 4, (int)MPU_Data.Acce.origin.z,4);
	
	HMC5883lRead(hmcdata);
	OLED_P6x8Str(64,2,"H-X");//Ð¡×Ö·û´®
	Dis_int(86, 2, (int)MPU_Data.Acce.origin.x,4); 
	OLED_P6x8Str(64,3,"H-Y");//Ð¡×Ö·û´®
	Dis_int(86, 3, (int)MPU_Data.Acce.origin.y,4);
	OLED_P6x8Str(64,4,"H-Z");//Ð¡×Ö·û´®
	Dis_int(86, 4, (int)MPU_Data.Acce.origin.z,4);
	
	delay_ms(500);
}

int main(void)
{
  Init();
	while(1)
	{
	 ;
	}
}
