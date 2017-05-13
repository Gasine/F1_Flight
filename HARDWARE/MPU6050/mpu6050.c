
#include "mpu6050.h"
struct MPU_Sensor MPU_Data;
int16_t MPU_Buf[12];     //Regard the original Register Data as signed int!! 
uint8_t Flag_GyroCali = 1;
uint8_t Flag_AcceCali = 1;

uint8_t MPU6050_Init(void)
{
	uint8_t ack;
	ack = Single_Read(MPU6050_ADDRESS, WHO_AM_I);
	if (!ack)
  return FALSE;
	
	Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	//解除休眠状态
	Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);     
	Single_Write(MPU6050_ADDRESS, CONFIGL, MPU6050_DLPF);              //低通滤波
	Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, MPU6050_GYRO_FS_1000);  //陀螺仪量程 +-1000
	Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, MPU6050_ACCEL_FS_4);   //加速度量程 +-4G	
	return TRUE;
}


void MPU6050_READ(void)
{
  MPU_Buf[0] = Single_Read(MPU6050_ADDRESS , ACCEL_XOUT_H);
	MPU_Buf[1] = Single_Read(MPU6050_ADDRESS , ACCEL_XOUT_L);
	MPU_Buf[2] = Single_Read(MPU6050_ADDRESS , ACCEL_YOUT_H);
	MPU_Buf[3] = Single_Read(MPU6050_ADDRESS , ACCEL_YOUT_L);
	MPU_Buf[4] = Single_Read(MPU6050_ADDRESS , ACCEL_ZOUT_H);
	MPU_Buf[5] = Single_Read(MPU6050_ADDRESS , ACCEL_ZOUT_L);
//	MPU_Buf[6] = Single_Read(MPU6050_ADDRESS , TEMP_OUT_H);
//	MPU_Buf[7] = Single_Read(MPU6050_ADDRESS , TEMP_OUT_L);
	MPU_Buf[6] = Single_Read(MPU6050_ADDRESS , GYRO_XOUT_H);
	MPU_Buf[7] = Single_Read(MPU6050_ADDRESS , GYRO_XOUT_L);
	MPU_Buf[8] = Single_Read(MPU6050_ADDRESS , GYRO_YOUT_H);
	MPU_Buf[9] = Single_Read(MPU6050_ADDRESS , GYRO_YOUT_L)	;
	MPU_Buf[10] = Single_Read(MPU6050_ADDRESS , GYRO_ZOUT_H);
	MPU_Buf[11] = Single_Read(MPU6050_ADDRESS , GYRO_ZOUT_L);
}


/***********************************************************
Description: Fuse the Register value of the MPU6050
H(8bit) + L(8bit)  ----->  Value(16bit)
************************************************************/
void Fuse_RegValue(void)
{

  MPU6050_READ();

	MPU_Data.Acce.origin.x =  (((int16_t)MPU_Buf[0]<<8) | (MPU_Buf[1])) - MPU_Data.Acce.quiet.x;
	MPU_Data.Acce.origin.y =  (((int16_t)MPU_Buf[2]<<8) | (MPU_Buf[3])) - MPU_Data.Acce.quiet.x;
	MPU_Data.Acce.origin.z =  (((int16_t)MPU_Buf[4]<<8) | (MPU_Buf[5]));
	MPU_Data.Gyro.origin.x =  (((int16_t)MPU_Buf[6]<<8) | (MPU_Buf[7]));
	MPU_Data.Gyro.origin.y =  (((int16_t)MPU_Buf[8]<<8) | (MPU_Buf[9]));
	MPU_Data.Gyro.origin.z =  (((int16_t)MPU_Buf[10]<<8) | (MPU_Buf[11]));
	
	MPU_Data.Gyro.radian.x = MPU_Data.Gyro.origin.x - MPU_Data.Gyro.quiet.x;
	MPU_Data.Gyro.radian.y = MPU_Data.Gyro.origin.y - MPU_Data.Gyro.quiet.y;
	MPU_Data.Gyro.radian.z = MPU_Data.Gyro.origin.z - MPU_Data.Gyro.quiet.z;
}

/************************************************************
Function Name: MPU6050_Cali
Description: Clibrate the MPU6050; Algorithm: Calculate The
queiscent value of MPU6050; Then Get rid of it(subtracting)
*************************************************************/
void MPU6050_Cali(void)   
{
	static uint8_t WthHorizontal;	
	uint8_t i = 0;
	uint8_t cnt = 0;
	int16_t *temp;
	int16_t *Integral;
	struct Axis3_int_16 TempLast;
  WthHorizontal  = 0;

	//Make sure the MPU6050 is steady and horizontal
	
	OLED_Fill(0x00); //clear screen
	OLED_P8x16Str(0,0, "Please Make Sure");//Display 8*16 english
	OLED_P8x16Str(0,2, "the Plane");//Display 8*16 english
	OLED_P8x16Str(0,4, "is Horizontal");//Display 8*16 english
	delay_ms(1500);
	delay_ms(1500);
	
	
	while(!WthHorizontal)
	{
		if(cnt<200)
		{
	    MPU6050_READ();
		
		  MPU_Data.Gyro.origin.x = (((int16_t)MPU_Buf[6] << 8) | MPU_Buf[7]);
		  MPU_Data.Gyro.origin.y = (((int16_t)MPU_Buf[8] << 8) | MPU_Buf[9]);
		  MPU_Data.Gyro.origin.z = (((int16_t)MPU_Buf[10] << 8) | MPU_Buf[11]);
			
			*(temp) += MPU_Data.Gyro.origin.x;
			*(temp+1) += MPU_Data.Gyro.origin.y;
			*(temp+2) += MPU_Data.Gyro.origin.z;
			
			*(Integral) += absu16(MPU_Data.Gyro.origin.x - TempLast.x); 
		  *(Integral+1) += absu16(MPU_Data.Gyro.origin.y - TempLast.y);
			*(Integral+2) += absu16(MPU_Data.Gyro.origin.z - TempLast.z);
			
			TempLast.x = MPU_Data.Gyro.origin.x;
			TempLast.y = MPU_Data.Gyro.origin.y;
			TempLast.z = MPU_Data.Gyro.origin.z;
		}
		//If the MPU6050 is not Steady and Horizontal
		else{
	   if((*(Integral) >= GYROX_Gather) || (*(Integral+1) >= GYROY_Gather) || (*(Integral+2) >= GYROZ_Gather))
			{
				cnt = 0;
				for(i=0;i<3;i++)
				{
					*(temp+i) = *(Integral+i) = 0;
				}
			}
			else
			{
				MPU_Data.Gyro.quiet.x = TempLast.x/200;
				MPU_Data.Gyro.quiet.y = TempLast.y/200;
				MPU_Data.Gyro.quiet.z = TempLast.z/200;
				WthHorizontal = 1;
				Flag_GyroCali = 0;
			}
		}
		cnt++;
	}
	//Calibrate the accelerometer
	if(Flag_AcceCali)
	{
		for(cnt = 0; cnt<200; cnt++)
		{
			MPU6050_READ();
			MPU_Data.Acce.origin.x = ((int16_t)MPU_Buf[0]<<8)|(MPU_Buf[1]);
			MPU_Data.Acce.origin.y = ((int16_t)MPU_Buf[2]<<8)|(MPU_Buf[3]);
			MPU_Data.Acce.origin.z = ((int16_t)MPU_Buf[4]<<8)|(MPU_Buf[5]);
			
			*(temp+3) += MPU_Data.Acce.origin.x;
			*(temp+4) += MPU_Data.Acce.origin.y;
			*(temp+5) += MPU_Data.Acce.origin.z;
		}
    MPU_Data.Acce.quiet.x = *(temp+3)/200;
    MPU_Data.Acce.quiet.y = *(temp+4)/200;	
		MPU_Data.Acce.quiet.z = *(temp+5)/200;
		Flag_AcceCali = 0;
	}
	
	OLED_Fill(0x00); //clear screen
	OLED_P8x16Str(0,0, "Calibration");//Display 8*16 english
	OLED_P8x16Str(0,2, "Complete");//Display 8*16 english
	//Display the zero offset
	delay_ms(1000);
	OLED_Fill(0x00); 
	Dis_int(0, 0, MPU_Data.Acce.quiet.x,5) ;//Display signed int
	Dis_int(0, 2, MPU_Data.Acce.quiet.y,5) ;//Display signed int
	Dis_int(0, 4, MPU_Data.Acce.quiet.z,5) ;//Display signed int
	delay_ms(1000);
	OLED_Fill(0x00); 
	
}

