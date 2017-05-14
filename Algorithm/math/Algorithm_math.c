
#include "Algorithm_math.h"

//#include "board_config.h"

/*====================================================================================================*/
/*====================================================================================================*
**���� : Q_rsqrt
**���� : ���ټ��� 1/Sqrt(x) 
**���� : number  
**��� : ���
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration ����һ��ţ�ٵ�����
	return y;
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:    array_astrict_lower(int16_t *array,int16_t value)
*��������:    ��������������
���������    *array   Ŀ������ָ��
*             value      
���������    ��
*******************************************************************************/
void array_astrict(s16 *array,s16 lower,s16 upper)
{
	 uint16_t i = 0;
   s16 length = sizeof(array); 
   for(i=0;i<length;i++)
   {
     if(*(array+i)<lower)  *(array+i) = lower;
     else if(*(array+i)>upper)  *(array+i) = upper;
   } 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    array_assign(int16_t *array,int16_t value)
*��������:    �����鸳ֵ
���������    *array   Ŀ������ָ�� 
*             value      
���������    ��
*******************************************************************************/
void array_assign(u16 *array,s16 value,u16 length)
{
	uint16_t i = 0;
   for(i=0;i<length;i++)
   {
     *(array+i) = value;
   } 
}

void array_assignu8(u8 *array,s16 value,u16 length)
{
	uint16_t i = 0;
   for(i=0;i<length;i++)
   {
     *(array+i) = value;
   } 
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:    data_limit(float data,flaot toplimit,float lowerlimit)
*��������:    �����޷�
���������    data       Ҫ���������� 
*             toplimit   ����
*             lowerlimit ����
���������    ��
*******************************************************************************/
float data_limit(float data,float toplimit,float lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
	return data;
}


/***********************************************
  * @brief  �ɱ���������Ӧ����
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.6f)
	{
	   error = 0.6f;
	}
	result = 1.0f - 1.667f * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:    rad(double angle)
*��������:    �Ƕ�ת��Ϊ����
���������    �Ƕ�
���������    ����
*******************************************************************************/
double Rad(double angle)
{
    return (angle * M_PI / 180.0);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:    degree(double rad)
*��������:    ����ת��Ϊ�Ƕ�	
���������    ����
���������    �Ƕ�
*******************************************************************************/
double Degree(double rad)
{
    return (rad / M_PI * 180.0);
}

double constrain(double inputvalue, double limitmin, double limitmax)
{
    if (inputvalue>limitmax) {
        inputvalue=limitmax;
    }
    else
    if (inputvalue<limitmin) {
        inputvalue=limitmin;
    }
		return inputvalue;
}

void applyDeadband(double value,double deadband)
{
  if((value  < deadband) && value > -deadband) 
  { value = 0;} 
  else if(value > 0)
        {value -= deadband;
        } else if(value < 0)
                {
                 value += deadband;
                }
}

float my_deathzoom_2(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}
  return (t);
}

/**************************�ж����������Ƿ����******************/
uint8_t Str_Equal(const char * incomp, char * inget, uint8_t length)
{
  int i = 0;
	for(i = 0; i<length; i++)
	{
	  if(*(incomp+i) == *(inget+i)) __NOP();
		else  break;
	}
	if (i == length)
	return TRUE;
	else
	return FALSE;
}

/**********************����ת�ַ���***************************/
char *IntToStr(u16 num, char *str)  
{  
    int i = 0, j = 0;  
    char *temp;  
    while(num)
    {  
        *(temp+i) = num % 10 + 48; 
        num = num / 10;  
        i++; 
    }  
    *(temp+i) = 0;    
      
    i = i - 1;  
    while(i >= 0)  
    {  
        *(str+j) = *(temp+i);  
        i--;  
        j++;  
    }  
    *(str+j) = 0;
    return str;  
}

char *gcvt(float number,uint8_t ndigits,char *buf)
{
  uint8_t i = 0;
	int numberint = 0;
	float numberfloat = 0.0;
	numberint = (int)number;
	while(numberint)
	{
		number = number/10;
		i++;
	}
	   IntToStr(numberint, buf);
		 buf[i+1] = '.';
		 numberfloat = (number - (float)numberint)*(ndigits);
		 numberint = (int)numberfloat;
		 IntToStr(numberint, &buf[i+2]);
	return buf;
}
