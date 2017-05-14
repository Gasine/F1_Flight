#ifndef __MPU6050_H_
#define __MPU6050_H_

#include <stdint.h>
#include "stm32f10x.h"
#include "iic.h"
#include "OLED.h"
#include "include.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
//****************************************
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIGL			  0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//IIC地址寄存器(默认数值0x68，只读)
#define MPU6050_ADDRESS 0xD0    //IIC写入时的地址字节数据，+1为读取

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06


    
#define MPU6050_DLPF  MPU6050_DLPF_BW_42 		//42Hz低通滤波

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x08
#define MPU6050_GYRO_FS_1000        0x10
#define MPU6050_GYRO_FS_2000        0x18
#define MPU6050_ACCEL_FS_2          0x01
#define MPU6050_ACCEL_FS_4          0x09
#define MPU6050_ACCEL_FS_8          0x11
#define MPU6050_ACCEL_FS_16         0x19



#define GYROX_Gather  300
#define GYROY_Gather  300
#define GYROZ_Gather  300

extern struct MPU_Sensor MPU_Data;
//Some Data Structure 
struct Axis3_int_16{
  int16_t x;
	int16_t y;
	int16_t z;
};

struct Axis3_float{
  float x;
	float y;
	float z;
};

struct Overall_Data{
  struct Axis3_int_16 origin;  //Original value
	struct Axis3_float average;  //Average value
	struct Axis3_float histor;   //Historical value
	struct Axis3_float radian;  //Radian value
	struct Axis3_int_16 quiet;  //Quiescent value
};

struct MPU_Sensor{
  struct Overall_Data Gyro;   // Data of the Gyroscope
	struct Overall_Data Acce;   //Data of the accelerometer
};



uint8_t   MPU6050_Init(void);			  
void MPU6050_READ(void);
void Fuse_RegValue(void);
void MPU6050_Cali(void);
#endif
