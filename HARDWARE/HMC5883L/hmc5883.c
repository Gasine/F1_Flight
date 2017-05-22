

#include "hmc5883.h"

//当前磁场的最大值和最小值
int16_t  HMC58X3_limit[6]={0};
int16_t  *mag_limt = HMC58X3_limit;
int16_t magdata[6]={0};

int16_t hmcdata[3] = {0};

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Init_HMC5883L
**功能 : 指南针初始化
**输入 : None
**輸出 : 状态
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
u8 Init_HMC5883L(void)
{
	u8 ack; 
	ack = Single_Read(MAG_ADDRESS, 0x0A);
	
	if (!ack)
			return FALSE;

	// leave test mode
	Single_Write(MAG_ADDRESS, HMC58X3_R_CONFA, 0x78);   // 0x70Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
	Single_Write(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   //0x20 Configuration Register B  -- 001 00000    configuration gain 1.33Ga
	Single_Write(MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
	delay_ms(100);
	
  flag.MagExist=1;
	return TRUE;	 
}
	
/*====================================================================================================*/
/*====================================================================================================*
**函数 : hmc5883lRead
**功能 : 度取地磁数据
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void HMC5883lRead(int16_t *magData)
{
	u8 buf[6],cy,con=0;
	int16_t mag[3];
	static u8 onc=1;
	static int32_t An[3] = {0,0,0};
	
	// 读取寄存器数据
	Multiple_Read(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);

	// 十位深度滤波
	An[0] -= An[0]/10;
	An[0] += (int16_t)(buf[0] << 8 | buf[1]);
	mag[0] = An[0]/10;

	An[1] -= An[1]/10;
	An[1] += (int16_t)(buf[4] << 8 | buf[5]);
	mag[1] = An[1]/10;

	An[2] -= An[2]/10;
	An[2] += (int16_t)(buf[2] << 8 | buf[3]);
	mag[2] = An[2]/10;
	
	
  magdata[0]=mag[0];
  magdata[1]=mag[1];
	magdata[2]=mag[2];

	//需要校准
	if(flag.calibratingM) {
		onc=1;
		flag.MagIssue = 0;
		Mag_Calibration(mag);
	}
 
	if(onc){
		onc=0;
		
		// 三个轴的最值都偏小 说明地磁有问题，停用地磁  
		for(cy=0;cy<6;cy++)	{
			if(absu16(*(mag_limt+cy))<20)	con++;
		}
		if(con>=2) flag.MagIssue = 1;
  }
	
	// 修正
	for(cy=0;cy<3;cy++)
		*(magData+cy) = (fp32)(mag[cy] -(*(mag_limt+cy+3) + *(mag_limt+cy))/2);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Mag_Calibration
**功能 : 地磁校准
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Mag_Calibration(int16_t *array)
{
	u8 cy;
	static u8  clen_flag=1; 
	static fp32 x,y,z; 
	
	//校准之前先把之前数据清零
	if(clen_flag){
		clen_flag = 0;
		x=y=z=0;
		for(cy=0;cy<6;cy++)
			*(mag_limt+cy)=0;
	}
  
	// 开始采集 寻找三个轴的最大和最小值
	for(cy=0;cy<3;cy++){
		if(*(mag_limt+cy)> *(array+cy)) *(mag_limt+cy) = *(array+cy);  //找最小

		else if(*(mag_limt+cy+3)<*(array+cy)) *(mag_limt+cy+3) = *(array+cy);  //找最大
	}
	//下面就是判断进行地磁校准的动作利用加速度计判断是否垂直，利用陀螺仪判断是否转满了360度
	if(flag.calibratingM == 1 && (absu16(MPU_Data.Acce.average.z) > 5000))   {
		GPIO_SetBits( GPIO_LED_RGB, LED_PIN_B | LED_PIN_G | LED_PIN_R);
	  GPIO_ResetBits( GPIO_LED_RGB, LED_PIN_B);
		OLED_P6x8Str(0,4,"rotate by z");
	  z += (double)MPU_Data.Gyro.radian.z * Gyro_G * 0.002f;  
		if(absFloat(z)>360)  
		{
			flag.calibratingM = 2;	
			OLED_Fill(0x00);		
			GPIO_SetBits( GPIO_LED_RGB, LED_PIN_B | LED_PIN_G | LED_PIN_R);
	    GPIO_ResetBits( GPIO_LED_RGB, LED_PIN_G);
		}
	}
	
	if(flag.calibratingM == 2 && (absu16(MPU_Data.Acce.average.x) > 5000))   {

		OLED_P6x8Str(0,4,"rotate by x");
	  x += (double)MPU_Data.Gyro.radian.x * Gyro_G * 0.002f;
		if(absFloat(x)>360)  
		{
			flag.calibratingM = 3;	
			GPIO_SetBits( GPIO_LED_RGB, LED_PIN_B | LED_PIN_G | LED_PIN_R);
		  GPIO_ResetBits( GPIO_LED_RGB, LED_PIN_R);
			OLED_Fill(0x00);
		}
	}
	
	if(flag.calibratingM == 3 && (absu16(MPU_Data.Acce.average.y) > 5000))   {

		OLED_P6x8Str(0,4,"rotate by y");
	  y += (double)MPU_Data.Gyro.radian.y * Gyro_G * 0.002f;
		if(absFloat(y)>360)  {
			OLED_Fill(0x00);
			flag.calibratingM = 0;
			GPIO_SetBits( GPIO_LED_RGB, LED_PIN_B | LED_PIN_G | LED_PIN_R);
			GPIO_ResetBits( GPIO_LED_RGB, LED_PIN_R | LED_PIN_G);
			OLED_Fill(0x00);
			flag.calicomplete = 1;
		}
	}	
}
