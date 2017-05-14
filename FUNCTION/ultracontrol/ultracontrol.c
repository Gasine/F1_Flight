/*****************************************
超声波定高模块
算法思想：
串级PID。内环Z方向上的速度。外环高度。采用
增量式PID控制器
******************************************/


#include "ultracontrol.h"

/******************高度控制变量********************/
float height_ctrl_out;
float wz_acc;
/*-------------------------------------------------*/


/******************超声波变量********************/
#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_INT        300    // 积分幅度
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


/******************速度环变量********************/
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
	
				if( MPU_Data.Acce.average.z > 7000 )//注意这里，指水平静态下的Z值要比这个大，表示水平垂直
				{
					  height_ctrl_start_f = 1;
				}
				break;
		
		case 1:
			
				LockForKeepHigh();//一旦切到控高模式，立即保存当前的距离值，油门值、相关清零。
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
		
				//userdata1[0]=	thr_lpf;//调试用
				/*下面的低通滤波用于测试对比效果，最终没有选用*/				
		    //wz_acc += ( 1 / ( 1 + 1 / ( 8 *3.14f *T ) ) ) *my_deathzoom( (V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y - wz_acc),100 );//
		    //wz_acc_mms2 = (float)(wz_acc/8192.0f) *9800 ;
				//加速度的静态零点是读取的EEPROM数据，但是每次飞行都可能不一样，能每次起飞前校准一次最好，可以单独校准Z轴的，自行设计程序		
				
				//test = V.z *(sensor.acc.averag.z  - sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y;
				
				wz_acc_temp = sqrt(abs(MPU_Data.Acce.average.x*MPU_Data.Acce.average.x) + abs(MPU_Data.Acce.average.y*MPU_Data.Acce.average.y) + abs(MPU_Data.Acce.average.z*MPU_Data.Acce.average.z)) - MPU_Data.Acce.quiet.z;	                                  
				
				Moving_Average( wz_acc_temp, acc_speed_arr, ACC_SPEED_NUM , acc_cnt ,&wz_acc1 );		
				
				tempacc_lpf = (float)(wz_acc1/8192.0f) *9800;//9800 *T;由于是+-4G共8G，65535/8g=8192 g，加速度，mms2毫米每平方秒		
				
				if(abs(tempacc_lpf)<50)tempacc_lpf=0;//简单消除下噪声	
				
				wz_speed_0 += tempacc_lpf *T;//加速度计积分成速度				
								
				if( ultra_start_f == 1 )////不管是啥模式，只要更新了超声波数据就进行运算
				{	
					 Ultra_dataporcess(15.0f*TT);
					 ultra_start_f=2;
				}
		
				if(flag.FlightMode==ULTRASONIC_High)
				{
					 h_speed = ultra_speed;
					 wz_speed_0 = 0.99f	*wz_speed_0 + 0.01f*h_speed;//超声波垂直速度互补滤波
				}				
				
				hc_speed_i += 0.4f *T *( h_speed - wz_speed );//速度偏差积分，乘以了0.4系数
				hc_speed_i = LIMIT( hc_speed_i, -500, 500 );//积分限幅
				
				wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;//0.1实测速度修正加速度算的速度
				
			  wz_speed = wz_speed_0 + hc_speed_i;//经过修正的速度+经过限幅的增量式速度积分.最终的垂直速度.测得的
				
				if( flag.FlightMode == ULTRASONIC_High)
				{
						height_speed_ctrl(T, ultra_ctrl_out, ultra_speed);//系数原来是0.4
					  
							
						if( ultra_start_f == 2 )//超声波数据更新了且被运算了
						{				
								Ultra_Ctrl(15.0f*TT,500);//realtime是周期为 TT 秒的，#define TT 0.0025//控制周期2.5ms
							  //outUltraPID();
								ultra_start_f = -1;
						}
				}
				
	      /*******************************************控制高度输出********************************************/
				if(flag.FlightMode==ULTRASONIC_High)//注意这里，模式
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
			  wz_speed_pid.kp = ultra_wz_speed_pid.kp;//变PID
		    wz_speed_pid.ki = ultra_wz_speed_pid.ki;
		    wz_speed_pid.kd = ultra_wz_speed_pid.kd;
        exp_height = ultra_dis_lpf;//这个超声波高度值经过滤波处理
			  wz_acc = 0;
			  wz_speed_pid_v.err_i = 0;
				ultra_speed = 0;//若不清零，则且超声波时原来累积的速度会错误传递给当前，造成掉高
			  ultra_ctrl.err_i = 0;
				ultra_ctrl.err_old = 0;
			  lock=1;
    }
}


void Ultra_dataporcess(float T)
{
		float ultra_sp_tmp,ultra_dis_tmp;	
	
		ultra_dis_tmp = Moving_Median(1,5,US100_Alt);//对超声波测量的距离进行移动中位值滤波
	
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
		
		//注意超声波测量的 距离的时效性问题，避免反复被计算
		ultra_sp_tmp = Moving_Median(0,5,US100_Alt_delta/T); //对超声波测距出来两次之间距离差计算的速度中值滤波 ultra_delta/T;

		if( ultra_dis_tmp < Ultrasonic_MAX_Height )//小于2.5米,注意这里，万一超出了呢？
		{
			if( ABS(ultra_sp_tmp) < 100 )//运动速度小于100mm/s
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ultra_speed会传递给速度环，作为当前速度的反馈,原来是4
			else
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//系数越大作用越小，原来是1
		}
}


void height_speed_ctrl(float T, float exp_z_speed, float h_speed)
{
		//wz_speed被用于参与超声波高度D运算
    wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );//
	
		wz_speed_pid_v.err_d = wz_speed_pid.kd * (-tempacc_lpf) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
		//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid_v.err *T;
		wz_speed_pid_v.err_i += wz_speed_pid.ki *( exp_z_speed - h_speed ) *T;//期望速度与实际速度误差积分
	
		wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i , -200, 200);
	
		wz_speed_pid_v.pid_out = LIMIT((wz_speed_pid.kp *exp_z_speed + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-300,300);//积分原来没有乘wz_speed_pid.kp
                               
		wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
}


void Ultra_PID_Init()
{
		ultra_pid.kp = 0.5;//超声波定高的高度位置环PID，会被EEPROM里的重新赋值的
		ultra_pid.ki = 0;
		ultra_pid.kd = 2.5;	//2.5
}


void Ultra_Ctrl(float T,float height)
{
		//exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 500,100)/300.0f; //20这里具体根据自己起飞油门定，油门控制高度+-ULTRA_SPEEDmm / s
	
//		exp_height_speed = LIMIT(exp_height_speed , -ULTRA_SPEED, ULTRA_SPEED);
//		
//		if( exp_height > Ultrasonic_MAX_Height )//超出超声波高度稳定范围则不执行
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
		
		exp_height = height ;//累积期望高度，因为期望速度可能改变了

		ultra_ctrl.err = ( ultra_pid.kp*(exp_height - ultra_dis_lpf) );//mm 对高度误差乘以Kp
		
			
		ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;//对高度误差积分
			
		ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i, -ULTRA_INT, ULTRA_INT);//积分限幅	
		
	  //对于D，融合了加速度计运算得到的距离  位置式PID控制。融合了加速度计的数据
		ultra_ctrl.err_d = ultra_pid.kd *( 0.5f *(-wz_speed*T) + 0.5f *(ultra_ctrl.err - ultra_ctrl.err_old) );
				
		ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;	
			
		//输出限制幅度
		ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-500,500);	
		
		//外环高度环控制。得到预期速度
		ultra_ctrl_out = ultra_ctrl.pid_out;
	
		ultra_ctrl.err_old = ultra_ctrl.err;
}

void WZ_Speed_PID_Init()
{
		ultra_wz_speed_pid.kp = 0.25;// //超声波定高的速度环PID，会被EEPROM里的重新赋值的
		ultra_wz_speed_pid.ki = 0.08; //0.12
		ultra_wz_speed_pid.kd = 8;
		wz_speed_pid.kp = 0.1;//速度环默认PID，真正用不到被重新赋值
		wz_speed_pid.ki = 0.08;
		wz_speed_pid.kd = 8;
}

