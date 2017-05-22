
//#include "board_config.h"
#include "ahrs.h"

#define KpDef 0.8f
#define KiDef 0.0005f
#define SampleRateHalf 0.00125f  //0.001

#define  IIR_ORDER     4      //ʹ��IIR�˲����Ľ���
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //ϵ��b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//ϵ��a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};
fp32 yawangle;
u16 testtime;


// /*	
// 	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
// 	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
// */
// #define KALMAN_Q        0.02
// #define KALMAN_R        8.0000

Quaternion NumQ = {1, 0, 0, 0};
EulerAngle AngE = {0},IMU = {0};


int16_t MAG[3];
Gravity V;//��������


void AHRS_getValues(void)
{
	static float x,y,z;
	
	Fuse_RegValue();
	
	HMC5883lRead(MAG);//260us
	
	// ���ٶȼ�IIR�˲�
	MPU_Data.Acce.average.x = IIR_I_Filter(MPU_Data.Acce.origin.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	MPU_Data.Acce.average.y = IIR_I_Filter(MPU_Data.Acce.origin.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	MPU_Data.Acce.average.z = IIR_I_Filter(MPU_Data.Acce.origin.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	
	// ������һ�׵�ͨ�˲�
 	MPU_Data.Gyro.average.x = LPF_1st(x,MPU_Data.Gyro.radian.x * Gyro_G,0.386f);	x = MPU_Data.Gyro.average.x;
 	MPU_Data.Gyro.average.y = LPF_1st(y,MPU_Data.Gyro.radian.y * Gyro_G,0.386f);	y = MPU_Data.Gyro.average.y;
 	MPU_Data.Gyro.average.z = LPF_1st(z,MPU_Data.Gyro.radian.z * Gyro_G ,0.386f);	z = MPU_Data.Gyro.average.z;//
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : AHRS_Update
**���� : AHRS
**���� : None
**��� : None
**ʹ�� : AHRS_Update();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_GetQ( Quaternion *pNumQ )
{
  fp32 ErrX, ErrY, ErrZ;
  fp32 AccX, AccY, AccZ;
  fp32 GyrX, GyrY, GyrZ;
	fp32 Normalize;
  static fp32 exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
	
	// ���ٶȹ�һ��
	Normalize = Q_rsqrt(squa(MPU_Data.Acce.average.x)+ squa(MPU_Data.Acce.average.y) +squa(MPU_Data.Acce.average.z));
	AccX = MPU_Data.Acce.average.x*Normalize;
  AccY = MPU_Data.Acce.average.y*Normalize;
  AccZ = MPU_Data.Acce.average.z*Normalize;

	// ��ȡ��������
	V = Quaternion_vectorGravity(&NumQ);
	
	// �������
 	ErrX = (AccY*V.z - AccZ*V.y);
  ErrY = (AccZ*V.x - AccX*V.z);
  ErrZ = (AccX*V.y - AccY*V.x);
 	
 	exInt = exInt + ErrX * KiDef;
  eyInt = eyInt + ErrY * KiDef;
  ezInt = ezInt + ErrZ * KiDef;

		
  GyrX = Rad(MPU_Data.Gyro.average.x) + KpDef * VariableParameter(ErrX) * ErrX  +  exInt; 
  GyrY = Rad(MPU_Data.Gyro.average.y) + KpDef * VariableParameter(ErrY) * ErrY  +  eyInt;
	GyrZ = Rad(MPU_Data.Gyro.average.z) + KpDef * VariableParameter(ErrZ) * ErrZ  +  ezInt; 
	
	
	// һ�����������, ������Ԫ��
	Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);
	
	// ��Ԫ����һ��
	Quaternion_Normalize(&NumQ);
	
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : AHRS_Geteuler
**���� : AHRS��ȡŷ����
**���� : None
**��� : None
**ʹ�� : AHRS_Geteuler();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_Geteuler(void)
{
	fp32 sin_pitch,sin_roll,cos_roll,cos_pitch;
	float digyaw;
	static float yawold;
	AHRS_getValues();
	
	// ��ȡ��Ԫ��
  AHRS_GetQ(&NumQ);
	
  // ��Ԫ��תŷ����
	Quaternion_ToAngE(&NumQ, &AngE);
	
  // ����ŷ���ǵ����Ǻ���ֵ
  sin_roll  = sin(AngE.Roll);
  sin_pitch = sin(AngE.Pitch);
  cos_roll  = cos(AngE.Roll);
  cos_pitch = cos(AngE.Pitch);
	
	//  �شŲ����ڻ�ش����ݲ�������ͣ�õش�����
//  flag.MagIssue=0;//�شŴ������⣬�����Խ��
//  flag.MagExist=0;

	if(!flag.MagIssue && flag.MagExist){//40US
		// �ش���ǲ���
		fp32 hx = MAG[0]*cos_pitch + MAG[1]*sin_pitch*sin_roll - MAG[2]*cos_roll*sin_pitch; 
		fp32 hy = MAG[1]*cos_roll + MAG[2]*sin_roll;
		
		// ���õشŽ��㺽���
		fp32 mag_yaw = -Degree(atan2((fp64)hy,(fp64)hx));
		 yawangle=mag_yaw;
		// �����ǻ��ֽ��㺽���
    //	AngE.Yaw += Degree(sensor.gyro.averag.z * Gyro_Gr * 2 * SampleRateHalf);//�ش�bug���Ѿ��ǽǶȣ����ﲻ���ٳ���Gyro_Gr��
		//AngE.Yaw += MPU_Data.Gyro.average.z * 2  * SampleRateHalf;
		// �شŽ���ĺ�����������ǻ��ֽ���ĺ���ǽ��л����ں� 
//		if((mag_yaw>90 && AngE.Yaw<-90) || (mag_yaw<-90 && AngE.Yaw>90)) 
//			IMU.Yaw = -(AngE.Yaw) * 0.012f + mag_yaw * 0.988f;
//		else 
//			IMU.Yaw = (AngE.Yaw) * 0.012f + mag_yaw * 0.988f;
	
    IMU.Yaw =  LPF_1st(yawold,mag_yaw,0.5f);
		
		yawold = IMU.Yaw;
	}
	else
		IMU.Yaw = Degree(AngE.Yaw);

  IMU.Roll = Degree(AngE.Roll);  // roll
	IMU.Pitch = Degree(AngE.Pitch); // pitch
}
