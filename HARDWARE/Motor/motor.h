#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "include.h"


#define Moto_PwmMax 1000

void TIM4_PWM_Init(u32 arr,u32 psc);
void moto_PwmRflash(u16 *Moter);
void moto_STOP(void);
void moto_Cali(void);
#endif
