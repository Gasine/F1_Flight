/*****************************************
����������ģ��
�㷨˼�룺
����PID���ڻ�Z�����ϵ��ٶȡ��⻷�߶ȡ�����
����ʽPID������
******************************************/


#include "ultracontrol.h"

/******************�߶ȿ��Ʊ���********************/
float height_ctrl_out;
float wz_acc;
/*-------------------------------------------------*/


/******************����������********************/
#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_INT        300    // ���ַ���
u8 lock=0;
extern s8 ultra_start_f;
extern float US100_Alt_delta;
float exp_height_speed,exp_height;
float ultra_speed,ultra_speed_acc;
float ultra_dis_lpf;
float ultra_ctrl_out;
_st_height_pid_v ultra_ctrl;
_st_height_pid ultra_pid;
/*-------------------------------------------------*/


/******************�ٶȻ�����********************/
float wz_speed;
float wz_acc_mms2;
float tempacc_lpf;
u8 lock_spd_crl=0;
_st_height_pid_v wz_speed_pid_v;
_st_height_pid wz_speed_pid;
_st_height_pid ultra_wz_speed_pid;
_st_height_pid baro_wz_speed_pid;
/*-------------------------------------------------*/


#define ACC_SPEED_NUM 50
float acc_speed_arr[ACC_SPEED_NUM + 1];
u16 acc_cnt[2];

void Height_Ctrl(float T)
{
	static u8 height_ctrl_start_f;
	static float wz_acc_temp,wz_acc1;
	static float hc_speed_i,wz_speed_0;
	static float h_speed;
	
	switch( height_ctrl_start_f )
	{
		case 0:
	
				if( MPU_Data.Acce.average.z > 7000 )//ע�����ָˮƽ��̬�µ�ZֵҪ������󣬱�ʾˮƽ��ֱ
				{
					  height_ctrl_start_f = 1;
				}
				break;
		
		case 1:
			
				LockForKeepHigh();//һ���е��ظ�ģʽ���������浱ǰ�ľ���ֵ������ֵ��������㡣
				if((flag.FlightMode == ULTRASONIC_High && lock == 1 &&lock_spd_crl == 0))
				{
						wz_speed_0=0;
						hc_speed_i=0;
						hc_speed_i=0;
						wz_speed=0;
						wz_speed_pid_v.err_old=0;
						wz_speed_pid_v.err=0;
						wz_speed_pid_v.err_i=0;
						lock=2;
						lock_spd_crl=1;
				}
		
				//userdata1[0]=	thr_lpf;//������
				/*����ĵ�ͨ�˲����ڲ��ԶԱ�Ч��������û��ѡ��*/				
		    //wz_acc += ( 1 / ( 1 + 1 / ( 8 *3.14f *T ) ) ) *my_deathzoom( (V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y - wz_acc),100 );//
		    //wz_acc_mms2 = (float)(wz_acc/8192.0f) *9800 ;
				//���ٶȵľ�̬����Ƕ�ȡ��EEPROM���ݣ�����ÿ�η��ж����ܲ�һ������ÿ�����ǰУ׼һ����ã����Ե���У׼Z��ģ�������Ƴ���		
				
				//test = V.z *(sensor.acc.averag.z  - sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y;
				
				wz_acc_temp = sqrt(abs(MPU_Data.Acce.average.x*MPU_Data.Acce.average.x) + abs(MPU_Data.Acce.average.y*MPU_Data.Acce.average.y) + abs(MPU_Data.Acce.average.z*MPU_Data.Acce.average.z)) - MPU_Data.Acce.quiet.z;	                                  
				
				Moving_Average( wz_acc_temp, acc_speed_arr, ACC_SPEED_NUM , acc_cnt ,&wz_acc1 );		
				
				tempacc_lpf = (float)(wz_acc1/8192.0f) *9800;//9800 *T;������+-4G��8G��65535/8g=8192 g�����ٶȣ�mms2����ÿƽ����		
				
				if(abs(tempacc_lpf)<50)tempacc_lpf=0;//������������	
				
				wz_speed_0 += tempacc_lpf *T;//���ٶȼƻ��ֳ��ٶ�				
								
				if( ultra_start_f == 1 )////������ɶģʽ��ֻҪ�����˳��������ݾͽ�������
				{	
					 Ultra_dataporcess(15.0f*TT);
					 ultra_start_f=2;
				}
		
				if(flag.FlightMode==ULTRASONIC_High)
				{
					 h_speed = ultra_speed;
					 wz_speed_0 = 0.99f	*wz_speed_0 + 0.01f*h_speed;//��������ֱ�ٶȻ����˲�
				}				
				
				hc_speed_i += 0.4f *T *( h_speed - wz_speed );//�ٶ�ƫ����֣�������0.4ϵ��
				hc_speed_i = LIMIT( hc_speed_i, -500, 500 );//�����޷�
				
				wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;//0.1ʵ���ٶ��������ٶ�����ٶ�
				
			  wz_speed = wz_speed_0 + hc_speed_i;//�����������ٶ�+�����޷�������ʽ�ٶȻ���.���յĴ�ֱ�ٶ�.��õ�
				
				if( flag.FlightMode == ULTRASONIC_High)
				{
						height_speed_ctrl(T, ultra_ctrl_out, ultra_speed);//ϵ��ԭ����0.4
					  
							
						if( ultra_start_f == 2 )//���������ݸ������ұ�������
						{				
								Ultra_Ctrl(15.0f*TT,500);//realtime������Ϊ TT ��ģ�#define TT 0.0025//��������2.5ms
							  //outUltraPID();
								ultra_start_f = -1;
						}
				}
				
	      /*******************************************���Ƹ߶����********************************************/
				if(flag.FlightMode==ULTRASONIC_High)//ע�����ģʽ
				{		
					  height_ctrl_out = wz_speed_pid_v.pid_out;
					  //outUltraSpPID();
				}
				break; 	
				default: break;
	} 
}


void LockForKeepHigh(void)
{
    if (flag.FlightMode==ULTRASONIC_High && lock==0)
		{
			  wz_speed_pid.kp = ultra_wz_speed_pid.kp;//��PID
		    wz_speed_pid.ki = ultra_wz_speed_pid.ki;
		    wz_speed_pid.kd = ultra_wz_speed_pid.kd;
        exp_height = ultra_dis_lpf;//����������߶�ֵ�����˲�����
			  wz_acc = 0;
			  wz_speed_pid_v.err_i = 0;
				ultra_speed = 0;//�������㣬���ҳ�����ʱԭ���ۻ����ٶȻ���󴫵ݸ���ǰ����ɵ���
			  ultra_ctrl.err_i = 0;
				ultra_ctrl.err_old = 0;
			  lock=1;
    }
}


void Ultra_dataporcess(float T)
{
		float ultra_sp_tmp,ultra_dis_tmp;	
	
		ultra_dis_tmp = Moving_Median(1,5,US100_Alt);//�Գ����������ľ�������ƶ���λֵ�˲�
	
		if( ultra_dis_tmp < Ultrasonic_MAX_Height )
		{	
			if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
			{	
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
			}
			else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
			{			
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
			else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
			{
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
			else
			{
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
		}
		
		//ע�ⳬ���������� �����ʱЧ�����⣬���ⷴ��������
		ultra_sp_tmp = Moving_Median(0,5,US100_Alt_delta/T); //�Գ���������������֮�����������ٶ���ֵ�˲� ultra_delta/T;

		if( ultra_dis_tmp < Ultrasonic_MAX_Height )//С��2.5��,ע�������һ�������أ�
		{
			if( ABS(ultra_sp_tmp) < 100 )//�˶��ٶ�С��100mm/s
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ultra_speed�ᴫ�ݸ��ٶȻ�����Ϊ��ǰ�ٶȵķ���,ԭ����4
			else
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ϵ��Խ������ԽС��ԭ����1
		}
}


void height_speed_ctrl(float T, float exp_z_speed, float h_speed)
{
		//wz_speed�����ڲ��볬�����߶�D����
    wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );//
	
		wz_speed_pid_v.err_d = wz_speed_pid.kd * (-tempacc_lpf) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
		//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid_v.err *T;
		wz_speed_pid_v.err_i += wz_speed_pid.ki *( exp_z_speed - h_speed ) *T;//�����ٶ���ʵ���ٶ�������
	
		wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i , -200, 200);
	
		wz_speed_pid_v.pid_out = LIMIT((wz_speed_pid.kp *exp_z_speed + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-300,300);//����ԭ��û�г�wz_speed_pid.kp
                               
		wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
}


void Ultra_PID_Init()
{
		ultra_pid.kp = 0.5;//���������ߵĸ߶�λ�û�PID���ᱻEEPROM������¸�ֵ��
		ultra_pid.ki = 0;
		ultra_pid.kd = 2.5;	//2.5
}


void Ultra_Ctrl(float T,float height)
{
		//exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 500,100)/300.0f; //20�����������Լ�������Ŷ������ſ��Ƹ߶�+-ULTRA_SPEEDmm / s
	
//		exp_height_speed = LIMIT(exp_height_speed , -ULTRA_SPEED, ULTRA_SPEED);
//		
//		if( exp_height > Ultrasonic_MAX_Height )//�����������߶��ȶ���Χ��ִ��
//		{
//			if( exp_height_speed > 0 )
//			{
//				exp_height_speed = 0;
//			}
//		}
//		else if( exp_height < Ultrasonic_MIN_Height )
//		{
//			if( exp_height_speed < 0 )
//			{
//				exp_height_speed = 0;
//			}
//		}
		
		exp_height = height ;//�ۻ������߶ȣ���Ϊ�����ٶȿ��ܸı���

		ultra_ctrl.err = ( ultra_pid.kp*(exp_height - ultra_dis_lpf) );//mm �Ը߶�������Kp
		
			
		ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;//�Ը߶�������
			
		ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i, -ULTRA_INT, ULTRA_INT);//�����޷�	
		
	  //����D���ں��˼��ٶȼ�����õ��ľ���  λ��ʽPID���ơ��ں��˼��ٶȼƵ�����
		ultra_ctrl.err_d = ultra_pid.kd *( 0.5f *(-wz_speed*T) + 0.5f *(ultra_ctrl.err - ultra_ctrl.err_old) );
				
		ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;	
			
		//������Ʒ���
		ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-500,500);	
		
		//�⻷�߶Ȼ����ơ��õ�Ԥ���ٶ�
		ultra_ctrl_out = ultra_ctrl.pid_out;
	
		ultra_ctrl.err_old = ultra_ctrl.err;
}

void WZ_Speed_PID_Init()
{
		ultra_wz_speed_pid.kp = 0.25;// //���������ߵ��ٶȻ�PID���ᱻEEPROM������¸�ֵ��
		ultra_wz_speed_pid.ki = 0.08; //0.12
		ultra_wz_speed_pid.kd = 8;
		wz_speed_pid.kp = 0.1;//�ٶȻ�Ĭ��PID�������ò��������¸�ֵ
		wz_speed_pid.ki = 0.08;
		wz_speed_pid.kd = 8;
}

