#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "ahrs.h"
#include "Para.h"
#include "motor.h"
#include "hc05.h"
#include "ultracontrol.h"
#define SIZE 9
extern u16 Moto[MOTOR_NUM];
extern char Moto1str[4];
extern struct _target target ;

struct _pid{
        float kp;
			  float ki;
	      float kd;
	      float increment;
	      float increment_max;
	      float kp_out;
			  float ki_out;
	      float kd_out;
	      float pid_out;
          };

struct _tache{
    struct _pid shell;
    struct _pid core;	
          };
	
struct _ctrl{
		      u8  ctrlRate;
      struct _tache pitch;    
	    struct _tache roll;  
	    struct _tache yaw;   
            };

struct _target{
      float Pitch;    
	    float Roll;  
	    float Yaw;   
            };

struct _keephigh_point{
      float current_THR;    
	    float current_ALT;  
	    float current_YAW;
			float current_PRESSURE;
            };						


extern struct _ctrl ctrl;
extern struct _target Target;

void Calculate_Target(void);
void CONTROL(struct _target Goal);
void Attitude_RatePID(void);
void Motor_Conter(void);
void Reset_Integral(void);
void Thr_Ctrl(float T);
void outAnglePID(void);
void outAngleSpPID(void);
void outUltraPID(void);
void outUltraSpPID(void);						
#endif
