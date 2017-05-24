
/*******************************************
This file is designed to create some #define
constants
*******************************************/
#ifndef __PARA_H
#define __PARA_H

/*--------------------�������----------------------*/
#define IDLING  300


#define MOTOR_NUM 4
/*------------------����ģʽ��������-------------------------*/
#define ULTRASONIC_High 1
#define MANUAL_High 2

/*-------------------����ģʽ����-------------------*/
#define SecondHigh_Factor 0.02       //������Ĳ���һ�����ڸ߶ȿ��ƻ����˲�
#define MainHigh_Factor   0.98

#define Ultrasonic_MAX_Height 2500   //�����������Ч�߶ȣ���λ��mm
#define Ultrasonic_MIN_Height 200    //�����������Ч�߶ȣ���λ��mm
#define Baro_MAX_Height       8000   //��ѹ�������Ч�߶ȣ���λ��mm,���Ը���ʵ���޸�
#define Baro_MIN_Height       1000   //��ѹ�������Ч�߶ȣ���λ��mm,�ٵ���ʵ��̫�ɿ������Ը���ʵ���޸�
#define TT                    0.0025f //��������2.5ms���붨ʱ��5�ж�ʱ���Ӧ
#define CTRL_HEIGHT           1      //0ʧ�ܣ�1ʹ�ܿظ߹���
#define TAKE_OFF_THR          550    //���ݸ��˷ɻ�ʵ��������������������ţ����ڶ���ģʽ�����������
#define MAX_PWM				        100		 //���PWM���Ϊ100%����
#define MAX_THR               80 		 //����ͨ�����ռ��80%����20%��������
typedef void (*rcReadRawData)(void);        


#endif 
