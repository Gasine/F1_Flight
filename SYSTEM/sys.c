#include "sys.h"


//关闭所有中断
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}

void delay_us(u32 nus)
{
	SysTick->LOAD=9*nus; //时间加载	  
	SysTick->CTRL=0x00000001;//打开定时器，采用系统时钟/8作为时钟来源
	while(!(SysTick->CTRL&0x00010000));//等待滴答定时器时间到达
	SysTick->CTRL=0x00000000;//关闭定时器
}
void delay_ms(u32 nms)
{
	SysTick->LOAD=9000*nms; //时间加载	  
	SysTick->CTRL=0x00000001;//打开定时器，采用系统时钟/8作为时钟来源
	while(!(SysTick->CTRL&0x00010000));//等待滴答定时器时间到达
	SysTick->CTRL=0x00000000;//关闭定时器
}
