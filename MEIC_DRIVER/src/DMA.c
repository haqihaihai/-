#include "DMA.h"

rc_t rc_control;
volatile unsigned char dma_memory[18]; 


void pa_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	GPIO_InitStructure.GPIO_Pin        		= GPIO_Pin_5 ; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode       		= GPIO_Mode_OUT;//���ù���
	GPIO_InitStructure.GPIO_Speed      		= GPIO_Speed_50MHz; //�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType 	 		= GPIO_OType_PP; //���츴�����
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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);	//ʹ��c,d����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//ʹ��USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);    //����DMAʱ��
	
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

		USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE); //ʹ�ܴ���1��DMA����
	}
	
	{
		  DMA_DeInit(DMA1_Stream0);
		  //����Stream
		  DMA_InitStructure.DMA_Channel 			= DMA_Channel_4;          //��8��channel��ѡ��һ��
		  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)&UART5->DR;            //�����ַ
		  DMA_InitStructure.DMA_Memory0BaseAddr 	= (u32)dma_memory;      //�洢��0��ַ��˫����ģʽ��Ҫʹ��M1AR
		  DMA_InitStructure.DMA_DIR 				= DMA_DIR_PeripheralToMemory;            //�洢��������ģʽ
		  DMA_InitStructure.DMA_BufferSize 			= 18;                //���ݴ�������������������Ϊ��λ 
		  DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;        //�����ַ���ֲ���
		  DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;                 //�洢����ַ����
		  DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte; //��������λ��:8λ
		  DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;         //�洢������λ��:8λ
		  DMA_InitStructure.DMA_Mode 				= DMA_Mode_Circular;                         //��ͨģʽ(��ѭ��ģʽ��Ӧ)
		  DMA_InitStructure.DMA_Priority 			= DMA_Priority_VeryHigh;                   //�е����ȼ�
		  DMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable;                  //��ֹFIFOģʽ         
		  DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
		  DMA_InitStructure.DMA_MemoryBurst 		= DMA_Mode_Normal;             //���δ���
		  DMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;     //���δ���
		  DMA_Init(DMA1_Stream0, &DMA_InitStructure);
		  DMA_Cmd(DMA1_Stream0, ENABLE); //ʹ��DMA

		  DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
		  //����DMA�ж����ȼ�
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
  rt_interrupt_enter();          //���ж���һ��Ҫ������Ժ����������ж�
	
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
  rt_interrupt_leave();    //���ж���һ��Ҫ������Ժ������뿪�ж� 
}

#if EN_UART5_RX   //���ʹ���˽���	
u8 UART5_RX_BUF[UART5_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 UART5_RX_STA=0;       //����״̬���	

void UART5_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
  /* enter interrupt */
	rt_interrupt_enter();          //���ж���һ��Ҫ������Ժ����������ж�
	
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(UART5);	//��ȡ���յ�������
		
		if((UART5_RX_STA&0x8000)==0)//����δ���
			{
			if(UART5_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)UART5_RX_STA=0;//���մ���,���¿�ʼ
				else UART5_RX_STA|=0x8000;	//��������� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)UART5_RX_STA|=0x4000;
				else
					{
					UART5_RX_BUF[UART5_RX_STA&0X3FFF]=Res ;
					UART5_RX_STA++;
					if(UART5_RX_STA>(UART5_REC_LEN-1))UART5_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
	
	/* leave interrupt */
  rt_interrupt_leave();    //���ж���һ��Ҫ������Ժ������뿪�ж� 
} 
#endif	


//void USART1_DMA_Init(void)//DMA2 Ch4 Stream2
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART2ʱ��
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);    //����DMAʱ�� 
//	
//{/***********************************����1����***********************************/
//	 //����1��Ӧ���Ÿ���ӳ��
//	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
//	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
//	 //USART1�˿�����
//	  GPIO_InitStructure.GPIO_Pin        		= GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
//	  GPIO_InitStructure.GPIO_Mode       		= GPIO_Mode_AF;//���ù���
//	  GPIO_InitStructure.GPIO_Speed      		= GPIO_Speed_50MHz; //�ٶ�50MHz
//	  GPIO_InitStructure.GPIO_OType 	 		= GPIO_OType_PP; //���츴�����
//	  GPIO_InitStructure.GPIO_PuPd 		 		= GPIO_PuPd_UP; //����
//	  GPIO_Init(GPIOA,&GPIO_InitStructure);
//	  USART_InitStructure.USART_BaudRate 		= 100000;//����������
//	  USART_InitStructure.USART_WordLength 		= USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	  USART_InitStructure.USART_StopBits 		= USART_StopBits_1;//һ��ֹͣλ
//	  USART_InitStructure.USART_Parity 			= USART_Parity_Even;//����żУ��λ
//	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	  USART_InitStructure.USART_Mode 			= USART_Mode_Rx ; //�շ�ģʽ
//	  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
//	  USART_Cmd(USART1, ENABLE);//ʹ�ܴ���1
//	
//	  //�����ж�
//	  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
//	  //����1NVIC����
//	  NVIC_InitStructure.NVIC_IRQChannel 		= USART1_IRQn;	 
//	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	  NVIC_Init(&NVIC_InitStructure);
//	  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE); //ʹ�ܴ���1��DMA����
//}		


//{/*************************************DMA����*************************************/
//	  DMA_DeInit(DMA2_Stream2);
//	  //����Stream
//	  DMA_InitStructure.DMA_Channel 			= DMA_Channel_4;          //��8��channel��ѡ��һ��
//	  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)&USART1->DR;            //�����ַ
//	  DMA_InitStructure.DMA_Memory0BaseAddr 	= (u32)dma_memory;      //�洢��0��ַ��˫����ģʽ��Ҫʹ��M1AR
//	  DMA_InitStructure.DMA_DIR 				= DMA_DIR_PeripheralToMemory;            //�洢��������ģʽ
//	  DMA_InitStructure.DMA_BufferSize 			= 18;                //���ݴ�������������������Ϊ��λ 
//	  DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;        //�����ַ���ֲ���
//	  DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;                 //�洢����ַ����
//	  DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte; //��������λ��:8λ
//	  DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;         //�洢������λ��:8λ
//	  DMA_InitStructure.DMA_Mode 				= DMA_Mode_Circular;                         //��ͨģʽ(��ѭ��ģʽ��Ӧ)
//	  DMA_InitStructure.DMA_Priority 			= DMA_Priority_VeryHigh;                   //�е����ȼ�
//	  DMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable;                  //��ֹFIFOģʽ         
//	  DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
//	  DMA_InitStructure.DMA_MemoryBurst 		= DMA_Mode_Normal;             //���δ���
//	  DMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;     //���δ���
//	  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
//	  DMA_Cmd(DMA2_Stream2, ENABLE); //ʹ��DMA

//	  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
//	  //����DMA�ж����ȼ�
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
//  rt_interrupt_enter();          //���ж���һ��Ҫ������Ժ����������ж�
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
//  rt_interrupt_leave();    //���ж���һ��Ҫ������Ժ������뿪�ж� 
//}

//#if EN_USART1_RX   //���ʹ���˽���	
//u8 USART1_RX_BUF[USART1_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//u16 USART1_RX_STA=0;       //����״̬���	

//void USART1_IRQHandler(void)                	//����1�жϷ������
//{
//	u8 Res;
//  /* enter interrupt */
//	rt_interrupt_enter();          //���ж���һ��Ҫ������Ժ����������ж�
//	
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//		{
//		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
//		
//		if((USART1_RX_STA&0x8000)==0)//����δ���
//			{
//			if(USART1_RX_STA&0x4000)//���յ���0x0d
//				{
//				if(Res!=0x0a)USART1_RX_STA=0;//���մ���,���¿�ʼ
//				else USART1_RX_STA|=0x8000;	//��������� 
//				}
//			else //��û�յ�0X0D
//				{	
//				if(Res==0x0d)USART1_RX_STA|=0x4000;
//				else
//					{
//					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=Res ;
//					USART1_RX_STA++;
//					if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//					}		 
//				}
//			}   		 
//     } 
//	
//	/* leave interrupt */
//  rt_interrupt_leave();    //���ж���һ��Ҫ������Ժ������뿪�ж� 
//} 
//#endif	



