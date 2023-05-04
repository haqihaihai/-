#include "DMA.h"

rc_t rc_control;
volatile unsigned char dma_memory[18]; 


void pa_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	GPIO_InitStructure.GPIO_Pin        		= GPIO_Pin_5 ; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode       		= GPIO_Mode_OUT;//复用功能
	GPIO_InitStructure.GPIO_Speed      		= GPIO_Speed_50MHz; //速度50MHz
	GPIO_InitStructure.GPIO_OType 	 		= GPIO_OType_PP; //推挽复用输出
	GPIO_Init(GPIOD,&GPIO_InitStructure);
}

void rc_init(void)
{
	rc_control.ch0=0;
	rc_control.ch1=0;
	rc_control.ch2=0;
	rc_control.ch3=0;
	rc_control.s1 =0;
	rc_control.s2 =0;
}


void UART5_DMA_INIT(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);	//使能c,d引脚
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//使能USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);    //开启DMA时钟
	
	{
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOC,&GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOD,&GPIO_InitStructure); 

		USART_DeInit(UART5);

		USART_InitStructure.USART_BaudRate = 100000;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(UART5, &USART_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

		USART_Cmd(UART5, ENABLE);

		USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE); //使能串口1的DMA发送
	}
	
	{
		  DMA_DeInit(DMA1_Stream0);
		  //配置Stream
		  DMA_InitStructure.DMA_Channel 			= DMA_Channel_4;          //从8个channel中选择一个
		  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)&UART5->DR;            //外设地址
		  DMA_InitStructure.DMA_Memory0BaseAddr 	= (u32)dma_memory;      //存储器0地址，双缓存模式还要使用M1AR
		  DMA_InitStructure.DMA_DIR 				= DMA_DIR_PeripheralToMemory;            //存储器到外设模式
		  DMA_InitStructure.DMA_BufferSize 			= 18;                //数据传输量，以外设数据项为单位 
		  DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;        //外设地址保持不变
		  DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;                 //存储器地址递增
		  DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte; //外设数据位宽:8位
		  DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;         //存储器数据位宽:8位
		  DMA_InitStructure.DMA_Mode 				= DMA_Mode_Circular;                         //普通模式(与循环模式对应)
		  DMA_InitStructure.DMA_Priority 			= DMA_Priority_VeryHigh;                   //中等优先级
		  DMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable;                  //禁止FIFO模式         
		  DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
		  DMA_InitStructure.DMA_MemoryBurst 		= DMA_Mode_Normal;             //单次传输
		  DMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;     //单次传输
		  DMA_Init(DMA1_Stream0, &DMA_InitStructure);
		  DMA_Cmd(DMA1_Stream0, ENABLE); //使能DMA

		  DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
		  //配置DMA中断优先级
		  NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream0_IRQn;           
		  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
		  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
		  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
		  NVIC_Init(&NVIC_InitStructure);
	}
	
}

void DMA1_Stream0_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断
	
	if(DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0)) 
	{ 
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0); 
		DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0); 

		rc_init();
		rc_control.ch0=(dma_memory[0] | (dma_memory[1] << 8)) & 0x07ff;
		rc_control.ch1=((dma_memory[1] >> 3) | (dma_memory[2] << 5)) & 0x07ff;
		rc_control.ch2=((dma_memory[2] >> 6) | (dma_memory[3] << 2) | (dma_memory[4] << 10))& 0x07ff;
		rc_control.ch3=((dma_memory[4] >> 1) | (dma_memory[5] << 7)) & 0x07ff;
		rc_control.s1=((dma_memory[5] >> 4) & 0x000C) >> 2;
		rc_control.s2=((dma_memory[5] >> 4) & 0x0003);
		
		rc_control.ch0-=1024;
		rc_control.ch1-=1024;
		rc_control.ch2-=1024;
		rc_control.ch3-=1024;
		//printf("ahfaislhfaihfoliashfahs");
	}
  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断 
}

#if EN_UART5_RX   //如果使能了接收	
u8 UART5_RX_BUF[UART5_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 UART5_RX_STA=0;       //接收状态标记	

void UART5_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
  /* enter interrupt */
	rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断
	
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(UART5);	//读取接收到的数据
		
		if((UART5_RX_STA&0x8000)==0)//接收未完成
			{
			if(UART5_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)UART5_RX_STA=0;//接收错误,重新开始
				else UART5_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)UART5_RX_STA|=0x4000;
				else
					{
					UART5_RX_BUF[UART5_RX_STA&0X3FFF]=Res ;
					UART5_RX_STA++;
					if(UART5_RX_STA>(UART5_REC_LEN-1))UART5_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
	
	/* leave interrupt */
  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断 
} 
#endif	


//void USART1_DMA_Init(void)//DMA2 Ch4 Stream2
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART2时钟
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);    //开启DMA时钟 
//	
//{/***********************************串口1配置***********************************/
//	 //串口1对应引脚复用映射
//	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
//	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
//	 //USART1端口配置
//	  GPIO_InitStructure.GPIO_Pin        		= GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
//	  GPIO_InitStructure.GPIO_Mode       		= GPIO_Mode_AF;//复用功能
//	  GPIO_InitStructure.GPIO_Speed      		= GPIO_Speed_50MHz; //速度50MHz
//	  GPIO_InitStructure.GPIO_OType 	 		= GPIO_OType_PP; //推挽复用输出
//	  GPIO_InitStructure.GPIO_PuPd 		 		= GPIO_PuPd_UP; //上拉
//	  GPIO_Init(GPIOA,&GPIO_InitStructure);
//	  USART_InitStructure.USART_BaudRate 		= 100000;//波特率设置
//	  USART_InitStructure.USART_WordLength 		= USART_WordLength_8b;//字长为8位数据格式
//	  USART_InitStructure.USART_StopBits 		= USART_StopBits_1;//一个停止位
//	  USART_InitStructure.USART_Parity 			= USART_Parity_Even;//无奇偶校验位
//	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	  USART_InitStructure.USART_Mode 			= USART_Mode_Rx ; //收发模式
//	  USART_Init(USART1, &USART_InitStructure); //初始化串口1
//	  USART_Cmd(USART1, ENABLE);//使能串口1
//	
//	  //串口中断
//	  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接收中断
//	  //串口1NVIC配置
//	  NVIC_InitStructure.NVIC_IRQChannel 		= USART1_IRQn;	 
//	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	  NVIC_Init(&NVIC_InitStructure);
//	  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE); //使能串口1的DMA发送
//}		


//{/*************************************DMA配置*************************************/
//	  DMA_DeInit(DMA2_Stream2);
//	  //配置Stream
//	  DMA_InitStructure.DMA_Channel 			= DMA_Channel_4;          //从8个channel中选择一个
//	  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)&USART1->DR;            //外设地址
//	  DMA_InitStructure.DMA_Memory0BaseAddr 	= (u32)dma_memory;      //存储器0地址，双缓存模式还要使用M1AR
//	  DMA_InitStructure.DMA_DIR 				= DMA_DIR_PeripheralToMemory;            //存储器到外设模式
//	  DMA_InitStructure.DMA_BufferSize 			= 18;                //数据传输量，以外设数据项为单位 
//	  DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;        //外设地址保持不变
//	  DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;                 //存储器地址递增
//	  DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte; //外设数据位宽:8位
//	  DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;         //存储器数据位宽:8位
//	  DMA_InitStructure.DMA_Mode 				= DMA_Mode_Circular;                         //普通模式(与循环模式对应)
//	  DMA_InitStructure.DMA_Priority 			= DMA_Priority_VeryHigh;                   //中等优先级
//	  DMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable;                  //禁止FIFO模式         
//	  DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
//	  DMA_InitStructure.DMA_MemoryBurst 		= DMA_Mode_Normal;             //单次传输
//	  DMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;     //单次传输
//	  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
//	  DMA_Cmd(DMA2_Stream2, ENABLE); //使能DMA

//	  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
//	  //配置DMA中断优先级
//	  NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream2_IRQn;           
//	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
//	  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
//	  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
//	  NVIC_Init(&NVIC_InitStructure);
//}

//}

//void DMA2_Stream2_IRQHandler(void)
//{
//  /* enter interrupt */
//  rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断
//	
//	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) 
//	{ 
//		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2); 
//		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); 

//		rc_init();
//		rc_control.ch0=(dma_memory[0] | (dma_memory[1] << 8)) & 0x07ff;
//		rc_control.ch1=((dma_memory[1] >> 3) | (dma_memory[2] << 5)) & 0x07ff;
//		rc_control.ch2=((dma_memory[2] >> 6) | (dma_memory[3] << 2) | (dma_memory[4] << 10))& 0x07ff;
//		rc_control.ch3=((dma_memory[4] >> 1) | (dma_memory[5] << 7)) & 0x07ff;
//		rc_control.s1=((dma_memory[5] >> 4) & 0x000C) >> 2;
//		rc_control.s2=((dma_memory[5] >> 4) & 0x0003);
//		
//		rc_control.ch0-=1024;
//		rc_control.ch1-=1024;
//		rc_control.ch2-=1024;
//		rc_control.ch3-=1024;
//		//printf("ahfaislhfaihfoliashfahs");
//	}
//  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断 
//}

//#if EN_USART1_RX   //如果使能了接收	
//u8 USART1_RX_BUF[USART1_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//u16 USART1_RX_STA=0;       //接收状态标记	

//void USART1_IRQHandler(void)                	//串口1中断服务程序
//{
//	u8 Res;
//  /* enter interrupt */
//	rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断
//	
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//		{
//		Res =USART_ReceiveData(USART1);	//读取接收到的数据
//		
//		if((USART1_RX_STA&0x8000)==0)//接收未完成
//			{
//			if(USART1_RX_STA&0x4000)//接收到了0x0d
//				{
//				if(Res!=0x0a)USART1_RX_STA=0;//接收错误,重新开始
//				else USART1_RX_STA|=0x8000;	//接收完成了 
//				}
//			else //还没收到0X0D
//				{	
//				if(Res==0x0d)USART1_RX_STA|=0x4000;
//				else
//					{
//					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=Res ;
//					USART1_RX_STA++;
//					if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收	  
//					}		 
//				}
//			}   		 
//     } 
//	
//	/* leave interrupt */
//  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断 
//} 
//#endif	



