
#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"

#define EnTIMER  TIM_Cmd(TIM5,ENABLE)
#define DisTIMER  TIM_Cmd(TIM5,DISABLE)

void TIM2_Init(u16 arr,u16 psc);
void TIM5_Config(void);
#endif
