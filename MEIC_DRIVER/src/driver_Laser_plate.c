//#include "driver_Laser_plate.h"
//#include "stdarg.h"	 
//#include "sys.h"
//#include "stdio.h"	 	 
//#include "string.h"	  

////���ڷ��ͻ����� 	
//__align(8) u8 UART5_TX_BUF[UART5_MAX_SEND_LEN]; 	//���ͻ���,���USART5_MAX_SEND_LEN�ֽ�
//__align(8) u8 UART5_RX_BUF[UART5_MAX_RECV_LEN]; 				//���ջ���,���USART5_MAX_RECV_LEN���ֽ�.
//float laser_Res[4]={'\0'};  //���յļ��ⷵ��ֵ

//#ifdef UART5_RX_EN   								//���ʹ���˽���   	  
////���ڽ��ջ����� 	
//u16 UART5_RX_STA=0;   	 
//int desTOP=0,desLOW=0; 
//u8 res;	   

//void UART5_IRQHandler(void)
//{

//	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)//���յ�����
//	{
////		printf("1");
//		res =USART_ReceiveData(UART5);
//		Laser_decode(res);
//  
//	}
//	 
//}




//#endif

//void uart5_init(u32 bound)
//{  
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure; 
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//ʹ��USART5ʱ��
//	
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
//	GPIO_Init(GPIOC,&GPIO_InitStructure); 
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
//	GPIO_Init(GPIOD,&GPIO_InitStructure); 
//	
//	USART_DeInit(UART5);
//	
//	USART_InitStructure.USART_BaudRate = bound;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No ;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(UART5, &USART_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
//	
//	USART_Cmd(UART5, ENABLE);
//	
//	UART5_RX_STA=0;

//}


///**	
//  * @brief 	����������ڽ����ж�����
//  * @note  	
//  * @param 	Res 	���봮�ڽ��յ�ֵ
//  * @retval	
//  */
//void Laser_decode(uint8_t Res)
//{
//	static u8 step=0;
//	static u8 length;
//	static UartOnlinDate temp_u4[4];
//	
//	switch(step)
//	{
//		case 0:	if(Res==0x0D)	step++;		else step=0; 		break;					//֡ͷ
//		case 1:	if(Res==0x0A) step++; 	else step=0; 	
//						if(Res==0x0D) step++;										break;
//		case 2:	length=Res; 																						//���� 
//						if(length > 7)	{step=0;  		break;}
//						if(length) 			step=3; else  step=7;		break;
//		case 3:	temp_u4[4-length]._u8[0]=Res;	step++;	  break;					//����
//		case 4:	temp_u4[4-length]._u8[1]=Res;	step++;		break;
//		case 5: temp_u4[4-length]._u8[2]=Res;	step++;		break;
//		case 6:	temp_u4[4-length]._u8[3]=Res;	step++;	
//						length --;
//						if(length) 		step=3; else step=7;		 	break;
//		case 7:	if(Res==0x0A)	step++;		else step=0;		break;					//֡β
//		case 8:	if(Res==0x0D) 	 			
//						{
//							laser_Res[0]= temp_u4[0]._fl_date;
//							laser_Res[1] = temp_u4[1]._fl_date;
//							laser_Res[2] = temp_u4[2]._fl_date;
//							laser_Res[3] = temp_u4[3]._fl_date;
//						} 
//						step=0;																	break;
//						
//		default: step=0; break;				
//	}
//}
//  




//void u5_printf(char* fmt,...)  
//{  
//	u16 i,j; 
//	va_list ap;
//	va_start(ap,fmt); 
//	vsprintf((char*)UART5_TX_BUF,fmt,ap);
//	va_end(ap);
//	i=strlen((const char*)UART5_TX_BUF);		
//	for(j=0;j<i;j++)					
//	{
//	  while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
//		USART_SendData(UART5,UART5_TX_BUF[j]);  
//	} 

//}



