#ifndef __MultiRotor_ahrs_H
#define	__MultiRotor_ahrs_H

#include "include.h"
#include "sys.h"
#include "Algorithm_quaternion.h"
#include "mpu6050.h"
#include "hmc5883.h"
#include "filter.h"
#define RtA 		57.324841f				
#define AtR    	0.0174533f				
#define Acc_G 	0.0011963f				
#define Gyro_G 	0.03051756f				
#define Gyro_Gr	0.0005426f



extern int16_t MAG[3];
//extern Gravity V;
void AHRS_getValues(void);
void AHRS_Geteuler(void);
#endif

