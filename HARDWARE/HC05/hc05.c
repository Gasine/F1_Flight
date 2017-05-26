#include "hc05.h"
u16 USART3_RX_STA=0;
u16 timeflag;
u16 time = 0;

void usart3_init(u32 bound)
{  
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	USART_DeInit(USART3);  //复位串口3
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
	
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);//开启重映射
	
  //USART3_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化GPIOE9，和GPIOE11
	  	
	//USART3_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = bound;//波特率一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断  
		
	USART_Cmd(USART3, ENABLE);                    //使能串口 
	
 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	USART3_RX_STA=0;				//清零 
}

//串口发送缓存区 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//发送缓冲,最大USART3_MAX_SEND_LEN字节  
//串口接收缓存区 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.


//通过判断接收连续2个字符之间的时间差不大于100ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过100ms,则认为不是1次连续数据.也就是超过100ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度

void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
	{	 
 
	res =USART_ReceiveData(USART3);	
	BluetoothDecode(res);
//	if((USART3_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
//	{ 
//		if(USART3_RX_STA<USART3_MAX_RECV_LEN)		//还可以接收数据
//		{   
//			time = 0;
//			if(USART3_RX_STA==0){
//				timeflag = 1;
//			}
//			USART3_RX_BUF[USART3_RX_STA++]=res;		//记录接收到的值	 
//		}else 
//		{
//			USART3_RX_STA|=1<<15;					//强制标记接收完成
//		} 
//	}
	
 }										 
} 

void u3_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);//此次发送数据的长度
	for(j=0;j<i;j++)//循环发送数据
	{
	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);  //等待上次传输完成 
		USART_SendData(USART3,(uint8_t)USART3_TX_BUF[j]); 	 //发送数据到串口3 
	}
	
}

void BluetoothDecode(u8 res)
{
	static u8 flagusart;
	static u8 buf[4];
	static u8 count;
	static u8 flagusart2,flagusart3,flagusart4,flagusart5,flagusart6;
	if(buf[1]=='o'&&buf[2]=='m')
	{
		buf[1]=0;
		buf[2]=0;
		switch (res)
		{
			case '1': flag.calibratingM = 1;break;
			case '2': flag.ARMED = 1;break;
			case '3': flag.ARMED = 0;break;
			case '4': flag.plus = 1;break;
			case '5': flag.minu = 1;break;
			case '6': moto_STOP();break;
			case '7': flag.FlightMode = MANUAL_High;break;
			case '8': target.Roll += 10;break;
//			case '9': target.Roll += 100;break;
			case 'a': target.Roll = 0;break;			
			default: break;
		}
	}
	if(res=='c')count=0,flagusart=1;
	if(flagusart==1)
	{
		buf[count]=res;
		count++;
		if(count==3)flagusart=0;
	}
	if(flagusart2==1){
		buf[count] =res;
		count++;
		if(count==3){
			ctrl.roll.core.kp =(((int)(buf[0]-'0'))*100 + ((int)(buf[1] -'0'))*10 +((int)(buf[2]-'0')))/100.0;
			flagusart2 = 0;
		}
	}
	if(res=='p'){
		count=0,flagusart2=1;
	}
	
	if(flagusart3==1){
		buf[count] =res;
		count++;
		if(count==3){
			ctrl.roll.core.kd =(((int)(buf[0]-'0'))*100 + ((int)(buf[1] -'0'))*10 +((int)(buf[2]-'0')))/100.0;
			u3_printf("d %f ",ctrl.roll.core.kd);
			flagusart3 = 0;
		}
	}
	if(res=='d'){
		count=0,flagusart3=1;
	}
	if(flagusart4==1){
		buf[count] =res;
		count++;
		if(count==3){
			ctrl.roll.core.ki =(((int)(buf[0]-'0'))*100 + ((int)(buf[1] -'0'))*10 +((int)(buf[2]-'0')))/100.0;
			u3_printf("i %f ",ctrl.roll.core.ki);
			flagusart4 = 0;
		}
	}
	if(res=='i'){
		count=0,flagusart4=1;
	}
	if(flagusart5==1){
		buf[count] =res;
		count++;
		if(count==3){
			ctrl.roll.shell.kp =(((int)(buf[0]-'0'))*100 + ((int)(buf[1] -'0'))*10 +((int)(buf[2]-'0')))/100.0;
			u3_printf("shell p %f ",ctrl.roll.shell.kp);
			flagusart5 = 0;
		}
	}
	if(res=='s'){
		count=0,flagusart5=1;
	}
	
	if(flagusart6==1){
		buf[count] =res;
		count++;
		if(count==3){
			ctrl.roll.shell.ki =(((int)(buf[0]-'0'))*100 + ((int)(buf[1] -'0'))*10 +((int)(buf[2]-'0')))/100.0;
			u3_printf("shell i %f ",ctrl.roll.shell.ki);
			flagusart6 = 0;
		}
	}
	if(res=='h'){
		count=0,flagusart6=1;
	}
}




//		USART3_RX_STA |= 1<<15;					//强制标记接收完成
//		timeflag = 0;
//		time = 0;
//		get = Str_Equal( "high" , USART3_RX_BUF,4);
//		if(get) situation = 0;
//		get = Str_Equal("stop", USART3_RX_BUF,4);
//		if(get) situation = 1;
//		get = Str_Equal("reset", USART3_RX_BUF,5);
//		if(get) situation = 3;
//		
//    switch(situation)
//		{
//		  case 0: 			
//			flag.FlightMode  = ULTRASONIC_High;
//      flag.ARMED  = 1; break;
//			case 1: 
//			flag.ARMED = 0; break;
//			case 3:
//			NVIC_SystemReset(); break;			
//			default: break;
//		}		
//		u3_printf(USART3_RX_BUF);
//		array_assignu8(USART3_RX_BUF,0x00,USART3_MAX_RECV_LEN);
//		USART3_RX_STA = 0;
