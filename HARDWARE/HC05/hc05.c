#include "hc05.h"
u16 USART3_RX_STA=0;
u16 timeflag;
u16 time = 0;

void usart3_init(u32 bound)
{  
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	USART_DeInit(USART3);  //��λ����3
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
	
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);//������ӳ��
	
  //USART3_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��GPIOE9����GPIOE11
	  	
	//USART3_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = bound;//������һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�  
		
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ��� 
	
 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	USART3_RX_STA=0;				//���� 
}

//���ڷ��ͻ����� 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�  
//���ڽ��ջ����� 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.


//ͨ���жϽ�������2���ַ�֮���ʱ������100ms�������ǲ���һ������������.
//���2���ַ����ռ������100ms,����Ϊ����1����������.Ҳ���ǳ���100msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���

void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
	{	 
 
	res =USART_ReceiveData(USART3);	
	BluetoothDecode(res);
//	if((USART3_RX_STA&(1<<15))==0)//�������һ������,��û�б�����,���ٽ�����������
//	{ 
//		if(USART3_RX_STA<USART3_MAX_RECV_LEN)		//�����Խ�������
//		{   
//			time = 0;
//			if(USART3_RX_STA==0){
//				timeflag = 1;
//			}
//			USART3_RX_BUF[USART3_RX_STA++]=res;		//��¼���յ���ֵ	 
//		}else 
//		{
//			USART3_RX_STA|=1<<15;					//ǿ�Ʊ�ǽ������
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
	i=strlen((const char*)USART3_TX_BUF);//�˴η������ݵĳ���
	for(j=0;j<i;j++)//ѭ����������
	{
	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������ 
		USART_SendData(USART3,(uint8_t)USART3_TX_BUF[j]); 	 //�������ݵ�����3 
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




//		USART3_RX_STA |= 1<<15;					//ǿ�Ʊ�ǽ������
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
