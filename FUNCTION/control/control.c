
#include "control.h"
struct _ctrl ctrl;
struct _target Target;

u16 Moto_duty[MOTOR_NUM];
u16 Moto[MOTOR_NUM];
u16 *motor_array = Moto_duty;
char Moto1str[4] = {0};
float thr_value;//����ģʽ�µ�����ֵ
u8 Thr_Low;
int date_throttle;
extern EulerAngle IMU;


/*====================================================================================================*/
/*====================================================================================================*
**���� : CONTROL(struct _target Goal) 
**���� : ����PID����
**���� : Goal
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void CONTROL(struct _target Goal)   
{
	float  deviation_pitch,deviation_roll,deviation_yaw;
	if(ctrl.ctrlRate >= 2)
	{////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//*****************�⻷(�ǶȻ�)PID**************************//
		//�������///////////////
	  deviation_pitch = Goal.Pitch - IMU.Pitch;
		ctrl.pitch.shell.increment += deviation_pitch;
		
		//limit for the max increment
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);

		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//��������//////////////
		deviation_roll = Goal.Roll - IMU.Roll;
		ctrl.roll.shell.increment += deviation_roll;
		
		//limit for the max increment
		ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);

		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
		
		//�������////////////
    if((Goal.Yaw - IMU.Yaw)>180 || (Goal.Yaw - IMU.Yaw)<-180){
       if(Goal.Yaw>0 && IMU.Yaw<0)  deviation_yaw= (-180 - IMU.Yaw) +(Goal.Yaw - 180);
       if(Goal.Yaw<0 && IMU.Yaw>0)  deviation_yaw= (180 - IMU.Yaw) +(Goal.Yaw + 180);
    }
    else  deviation_yaw = Goal.Yaw - IMU.Yaw;
		
	  ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * deviation_yaw;
    ctrl.ctrlRate = 0;
		//outAnglePID();
	  
  }
	ctrl.ctrlRate ++;
  Attitude_RatePID();
	Thr_Ctrl(TT);// ���ſ���
	Motor_Conter();
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : Attitude_RatePID
**���� : �����ʿ���PID
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(void)
{
  fp32 E_pitch,E_roll,E_yaw;
	
	// ����ƫ��  
	//E_pitch = target.Pitch -  MPU_Data.Gyro.average.y;
	E_pitch = ctrl.pitch.shell.pid_out - MPU_Data.Gyro.average.y;
	E_yaw   = ctrl.yaw.shell.pid_out   - MPU_Data.Gyro.average.z;
  //E_roll = target.Roll - MPU_Data.Gyro.average.x;
	E_roll = ctrl.roll.shell.pid_out - MPU_Data.Gyro.average.x;
	// ����
	ctrl.pitch.core.increment += E_pitch;
	ctrl.roll.core.increment  += E_roll;
	ctrl.yaw.core.increment   += E_yaw;
	
	// �����޷�
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;
	
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
  ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	// ΢��  
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (MPU_Data.Gyro.histor.y - MPU_Data.Gyro.average.y)*33;
	ctrl.roll.core.kd_out  = ctrl.roll.core.kd  * (MPU_Data.Gyro.histor.x - MPU_Data.Gyro.average.x)*33;
	ctrl.yaw.core.kd_out   = ctrl.yaw.core.kd   * (MPU_Data.Gyro.histor.z - MPU_Data.Gyro.average.z)*33;	
	
	MPU_Data.Gyro.histor.y = MPU_Data.Gyro.average.y;
	MPU_Data.Gyro.histor.x = MPU_Data.Gyro.average.x; 
  MPU_Data.Gyro.histor.z = MPU_Data.Gyro.average.z;	
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.roll.core.pid_out  = ctrl.roll.core.kp_out  + ctrl.roll.core.ki_out  + ctrl.roll.core.kd_out;
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.kp_out   + ctrl.yaw.core.kd_out;
	
	//ctrl.pitch.core.pid_out = ctrl.pitch.core.pid_out*0.8f + ctrl.pitch.shell.pid_out/2;
	//ctrl.roll.core.pid_out  = ctrl.roll.core.pid_out *0.8f + ctrl.roll.shell.pid_out/2; 
	//ctrl.yaw.core.pid_out   = ctrl.yaw.core.pid_out;
	
	//outAngleSpPID();

}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Motor_Conter(void)
**���� : �������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Motor_Conter(void)
{
	  s16 pitch,roll,yaw;
	  u8 i=0;
	
		pitch = ctrl.pitch.core.pid_out;
		roll  = ctrl.roll.core.pid_out;    
		yaw   = -ctrl.yaw.core.pid_out;
	
  	if((flag.FlightMode == ULTRASONIC_High) || (flag.FlightMode == MANUAL_High)){           
			Moto[0] =  - roll /*thr_value + yaw*/- pitch + IDLING;
			Moto[1] =  + roll /*thr_value  - yaw */- pitch+ IDLING;
			Moto[2] =  + roll /*thr_value  + yaw */+ pitch+ IDLING;
			Moto[3] =  - roll /*thr_value - yaw*/ + pitch+ IDLING;
    }
		else
		{	
			array_assign(&Moto[0],IDLING,MOTOR_NUM);//������300
			Reset_Integral();//�ڻ�pidȫ�������0		
		}
		
		if(flag.ARMED)
		{		
					for(i=0;i<4;i++)
			    {
							Moto_duty[i]=Moto[i];
					}			
			moto_PwmRflash(&Moto_duty[0]);//������ˢ�£�ֱ��дPWM����Ĵ���
					
//		  IntToStr((Moto_duty[1]+999), Moto1str); 
//			u3_printf(Moto1str);
//			u3_printf("   ");
		}	
		else 
		{
			 array_assign(&Moto_duty[0],0,MOTOR_NUM);//������0
		   moto_STOP();//ǿ�����1000			
		}				
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : Reset_Integral
**���� : ��������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Reset_Integral(void)
{
	ctrl.pitch.shell.increment = 0;
	ctrl.roll.shell.increment= 0;	
  ctrl.pitch.core.increment = 0;		
  ctrl.roll.core.increment = 0;		
	ctrl.yaw.core.increment = 0;
}

void Thr_Ctrl(float T)
{
	Height_Ctrl(T);
	
	thr_value = height_ctrl_out;   //ʵ��ʹ��ֵ
	
	thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);//�����������Ϊ800����200��ظ���̬����
}

void outAnglePID(void)
{
	char OUT[SIZE];
	gcvt( ctrl.pitch.shell.pid_out ,3, OUT);
	u3_printf("PID shell out pitch:");
	u3_printf(OUT);
	u3_printf("\n");
	
	gcvt( ctrl.roll.shell.pid_out ,3, OUT);
	u3_printf("PID shell out roll:");
	u3_printf(OUT);
	u3_printf("\n");
	
	gcvt( ctrl.yaw.shell.pid_out ,3, OUT);
	u3_printf("PID shell out yaw:");
	u3_printf(OUT);
	u3_printf("\n");
}


void outAngleSpPID(void)
{
 	char OUT[SIZE];
	gcvt( ctrl.pitch.core.pid_out ,3, OUT);
	u3_printf("PID core out pitch:");
	u3_printf(OUT);
	u3_printf("\n");
	
	gcvt( ctrl.roll.core.pid_out ,3, OUT);
	u3_printf("PID core out roll:");
	u3_printf(OUT);
	u3_printf("\n");
	
	gcvt( ctrl.yaw.core.pid_out ,3, OUT);
	u3_printf("PID core out yaw:");
	u3_printf(OUT);
	u3_printf("\n");
}


void outUltraPID(void)
{
  char OUT[SIZE];
	gcvt( ultra_ctrl_out ,3, OUT);
	u3_printf("PID ultra out:");
	u3_printf(OUT);
	u3_printf("\n");
}


void outUltraSpPID(void)
{
  char OUT[SIZE];
	gcvt(height_ctrl_out ,3, OUT);
	u3_printf("PID ultra out:");
	u3_printf(OUT);
	u3_printf("\n");
}
