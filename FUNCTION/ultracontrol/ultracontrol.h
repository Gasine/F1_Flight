#ifndef __ULTRACONTROL_H
#define __ULTRACONTROL_H

#include "sys.h"
#include "include.h"
#include "mpu6050.h"
#include "hmc5883.h"
#include "Para.h"
#include <math.h>
#include "Algorithm_math.h"
#include <stdlib.h>
#include "filter.h"
#include "Ultrasonic.h"
typedef struct
{
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;

}_st_height_pid_v;

typedef struct
{
	float kp;
	float kd;
	float ki;

}_st_height_pid;

void Height_Ctrl(float T);
void Ultra_PID_Init(void);
void WZ_Speed_PID_Init(void);
void height_speed_ctrl(float T, float exp_z_speed,float h_speed );
void Ultra_Ctrl(float T,float thr);
void LockForKeepHigh(void);
void Ultra_dataporcess(float T);

extern float ultra_ctrl_out;
extern float height_ctrl_out;
extern float ultra_speed, ultra_dis_lpf;
extern _st_height_pid  wz_speed_pid;
extern _st_height_pid  ultra_pid;
#endif
