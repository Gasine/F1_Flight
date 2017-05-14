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
/*------------------������ƫ����--------------------*/
/*------------------��Ҫ  ��Ҫ��--------------------*/
#define FLASH_EXCURSION  0x20000
#define pro_FALG_ADD     0x0801FFF0

typedef float fp32;
typedef double fp64;


typedef struct {
	      unsigned char MpuExist;      // MPU����
	      unsigned char MagExist;      // MAG����
	      unsigned char NrfExist;      // NRF����
	      unsigned char MagIssue;      // MAG������
        unsigned char MsExist;
        unsigned char ARMED;         // �������
	      unsigned char LockYaw;       // ��������  
        unsigned char calibratingR;	// ң����У׼
        unsigned char calibratingA;  // ���ٶȲɼ�
	      unsigned char calibratingM;  // �����Ʋɼ�
	      unsigned char calibratingM_pre; //������Ԥ�ɼ�
	      unsigned char calibratingG;
	      unsigned char ParamSave;     // ���������־
	
	      unsigned char Loop_200Hz;
	      unsigned char Loop_100Hz;
				unsigned char Loop_40Hz;
				unsigned char Loop_27Hz;
				unsigned char Loop_20Hz;
	      unsigned char Loop_10Hz;
        unsigned char fortest;//������ʱ��
				unsigned char FlightMode;
				unsigned char HUDMode;
				unsigned char ControlMode;
				unsigned char Special_Mode;
				
				unsigned char calicomplete;
         }Flag_t;
extern Flag_t flag;
				 

#endif
