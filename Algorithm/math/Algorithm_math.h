#ifndef __Algorithm_math_H
#define	__Algorithm_math_H

#include "sys.h"
#include "include.h"

#define M_PI  (float)3.1415926535
#define squa( Sq )        (((float)Sq)*((float)Sq))
#define toRad( Math_D )	  ((float)(Math_D)*0.0174532925f)
#define toDeg( Math_R )	  ((float)(Math_R)*57.2957795f)
#define absu16( Math_X )  (Math_X<0? -(Math_X):Math_X)
#define absFloat( Math_X )(Math_X<0? -(Math_X):Math_X)

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

void array_assignu8(u8 *array,s16 value,u16 length);
float my_deathzoom_2(float x,float zoom);
float Q_rsqrt(float number);
float VariableParameter(float error);
void array_assign(u16 *array,s16 value,u16 length);
void array_astrict(s16 *array,s16 lower,s16 upper);
float data_limit(float data,float toplimit,float lowerlimit);
double Degree(double rad);
double Rad(double angle);
double constrain(double inputvalue, double limitmin, double limitmax);
void applyDeadband(double value,double deadband);
uint8_t Str_Equal(const char * incomp, char * inget, uint8_t length);
char *IntToStr(uint16_t num, char *str);
char *gcvt(float number,uint8_t ndigits,char *buf);
#endif /* __Algorithm_math_H */
