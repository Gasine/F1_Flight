#ifndef _INCLUDE_
#define  _INCLUDE_

#define TRUE 1
#define FALSE 0
#define absu16(Math_X)  (Math_X<0? -(Math_X):Math_X)
#define absFloat(Math_X)(Math_X<0? -(Math_X):Math_X)
#define RtA 		57.324841f				
#define AtR    	0.0174533f				
#define Acc_G 	0.0011963f				
#define Gyro_G 	0.03051756f				
#define Gyro_Gr	0.0005426f
#define MOTOR_NUM 4

typedef float fp32;
typedef double fp64;


typedef struct {
	      unsigned char MpuExist;      // MPU存在
	      unsigned char MagExist;      // MAG存在
	      unsigned char NrfExist;      // NRF存在
	      unsigned char MagIssue;      // MAG有问题
        unsigned char MsExist;
        unsigned char ARMED;         // 电机解锁
	      unsigned char LockYaw;       // 航向锁定  
        unsigned char calibratingR;	// 遥控器校准
        unsigned char calibratingA;  // 加速度采集
	      unsigned char calibratingM;  // 磁力计采集
	      unsigned char calibratingM_pre; //磁力计预采集
	      unsigned char calibratingG;
	      unsigned char ParamSave;     // 参数保存标志
	
	      unsigned char Loop_200Hz;
	      unsigned char Loop_100Hz;
				unsigned char Loop_40Hz;
				unsigned char Loop_27Hz;
				unsigned char Loop_20Hz;
	      unsigned char Loop_10Hz;
        unsigned char fortest;//加来临时用
				unsigned char FlightMode;
				unsigned char HUDMode;
				unsigned char ControlMode;
				unsigned char Special_Mode;
         }Flag_t;
extern Flag_t flag;
				 

#endif
