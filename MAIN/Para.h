
/*******************************************
This file is designed to create some #define
constants
*******************************************/
#ifndef __PARA_H
#define __PARA_H

/*--------------------电机怠速----------------------*/
#define IDLING  300


#define MOTOR_NUM 4
/*------------------飞行模式参数设置-------------------------*/
#define ULTRASONIC_High 1
#define MANUAL_High 2

/*-------------------定高模式参数-------------------*/
#define SecondHigh_Factor 0.02       //与下面的参数一起用于高度控制互补滤波
#define MainHigh_Factor   0.98

#define Ultrasonic_MAX_Height 2500   //超声波最高有效高度，单位是mm
#define Ultrasonic_MIN_Height 200    //超声波最低有效高度，单位是mm
#define Baro_MAX_Height       8000   //气压计最高有效高度，单位是mm,可以根据实际修改
#define Baro_MIN_Height       1000   //气压计最低有效高度，单位是mm,再低其实不太可靠，可以根据实际修改
#define TT                    0.0025f //控制周期2.5ms，与定时器5中断时间对应
#define CTRL_HEIGHT           1      //0失能，1使能控高功能
#define TAKE_OFF_THR          550    //根据个人飞机实际情况，设置起飞离地油门，用于定高模式下稍推油起飞
#define MAX_PWM				        100		 //最大PWM输出为100%油门
#define MAX_THR               80 		 //油门通道最大占比80%，留20%给控制量
typedef void (*rcReadRawData)(void);        


#endif 
