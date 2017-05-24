#ifndef _HC05_H_

#define _HC05_H_


#include "include.h"
#include "sys.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
extern u16 timeflag,time,USART3_RX_STA;
#define USART3_MAX_RECV_LEN		400					//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		400					//����ͻ����ֽ���

extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
void BluetoothDecode(u8 res);
void usart3_init(u32 bound);
void u3_printf(char* fmt, ...);




#endif
