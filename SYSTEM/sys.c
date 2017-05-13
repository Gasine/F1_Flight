#include "sys.h"


//�ر������ж�
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//���������ж�
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}

void delay_us(u32 nus)
{
	SysTick->LOAD=9*nus; //ʱ�����	  
	SysTick->CTRL=0x00000001;//�򿪶�ʱ��������ϵͳʱ��/8��Ϊʱ����Դ
	while(!(SysTick->CTRL&0x00010000));//�ȴ��δ�ʱ��ʱ�䵽��
	SysTick->CTRL=0x00000000;//�رն�ʱ��
}
void delay_ms(u32 nms)
{
	SysTick->LOAD=9000*nms; //ʱ�����	  
	SysTick->CTRL=0x00000001;//�򿪶�ʱ��������ϵͳʱ��/8��Ϊʱ����Դ
	while(!(SysTick->CTRL&0x00010000));//�ȴ��δ�ʱ��ʱ�䵽��
	SysTick->CTRL=0x00000000;//�رն�ʱ��
}
