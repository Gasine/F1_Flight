#ifndef __Ultrasonic_H_
#define __Ultrasonic_H_

#include "sys.h"
#include <stdlib.h>


extern float US100_Alt;
extern float US100_Alt_delta;
extern u8 Ultrasonic_OK;
extern s8 ultra_start_f;
extern u16 Drop,Rise;
extern vu16 US100_Alt_Temp,Alt_Last; 
extern float US100_Alt_Last;

void Ultrasonic_Config(void);
void Ultrasonic_Pulsing(void);
void Ultra_dataporcess(float T);
void PWM_IN_Config(void);
#endif // __Ultrasonic_H__
