#include "sys.h"

static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数
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

/**************************实现函数********************************************
*函数原型:		void delay_init(u8 SYSCLK)
*功　　能:		初始化延迟系统，使延时程序进入可用状态
*******************************************************************************/
void delay_init(u8 SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
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
