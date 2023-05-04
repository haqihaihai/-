#include "DRIVER_DBUS.h"
#include "usart_go.h"

DMA_InitTypeDef DMA_InitStructure;

MOTOR_REC_T motor_buffer6;
MOTOR_REC_T motor_buffer1;

void DBUS_Init(void)
{

	/* -------------- Enable Module Clock Source ----------------------------*/
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	/* -------------- Configure GPIO ---------------------------------------*/

		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure; 
		
		GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_14; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOG,&GPIO_InitStructure); 
		
		USART_DeInit(USART6);
		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
		
		USART_InitStructure.USART_BaudRate = 4000000;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART6, &USART_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;	 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
		
		USART_Cmd(USART6, ENABLE);


	//DMA2 stream5 ch4  or (DMA2 stream2 ch4)    !!!!!!! P206 of the datasheet
	/* -------------- Configure DMA -----------------------------------------*/
	{
	//  DMA_InitTypeDef DMA_InitStructure;
		
	  DMA_DeInit(DMA2_Stream1);
	  while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
	  DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&motor_buffer6.buf[0];
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStructure.DMA_BufferSize = 17;
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;

	  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	  DMA_Cmd(DMA2_Stream1, ENABLE);
	  
	}
	  USART_Cmd(USART6, ENABLE);
}

void DBUS_init_2(void)
{
		/* -------------- Enable Module Clock Source ----------------------------*/
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* -------------- Configure GPIO ---------------------------------------*/

		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure; 
		
	{
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOA,&GPIO_InitStructure); 
		
		USART_DeInit(USART1);
		
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
		
		USART_InitStructure.USART_BaudRate = 4000000;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
		
		USART_Cmd(USART1, ENABLE);
	}

	//DMA2 stream5 ch4  or (DMA2 stream2 ch4)    !!!!!!! P206 of the datasheet
	/* -------------- Configure DMA -----------------------------------------*/
	{
	//  DMA_InitTypeDef DMA_InitStructure;
		
	  DMA_DeInit(DMA2_Stream2);
	  while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
	  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&motor_buffer1.buf[0];
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStructure.DMA_BufferSize = 17;
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;

	  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(DMA2_Stream2, &DMA_InitStructure);

	  DMA_Cmd(DMA2_Stream2, ENABLE);
	  
	}
	  USART_Cmd(USART1, ENABLE);
}

extern MOTOR_REC_T motor_receive_go8[8];

void remote_control_init(void)
{
	
    DBUS_Init();
}

void remote_control_init2(void)
{
	DBUS_init_2();
}

int flag_zero_1=0;
int flag_zero_2=0;
int flag_zero=0;

void USART1_IRQHandler(void)
{
	rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断

	u16 data;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //
	{
		USART_ClearITPendingBit(USART1,USART_IT_IDLE); // 清除标志位
		DMA_Cmd(DMA2_Stream2, DISABLE);
		data = USART1->SR;
		data = USART1->DR;  	// 这里必须读一下串口的SR和DR寄存器  这样程序才能正常运行
		DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF4 | DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4);
		while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
		//printf("motor%d %d\r\n",motor_buffer1.rec_data.rad,motor_buffer1.rec_data.rads);
		//printf("crc%d %d\r\n",motor_buffer1.rec_data.crc,crc_ccitt(0,motor_buffer1.buf,14));
		if(motor_buffer1.rec_data.crc ==crc_ccitt(0,motor_buffer1.buf,14))
        {
			
			//printf("enter");
			switch(motor_buffer1.rec_data.id)
			{
					case 0: for(int i=0;i<16;i++){motor_receive_go8[0].buf[i]=motor_buffer1.buf[i];}break;
					case 1:	for(int i=0;i<16;i++){motor_receive_go8[1].buf[i]=motor_buffer1.buf[i];}break;
//					case 2: for(int i=0;i<16;i++){motor_receive_go8[2].buf[i]=motor_buffer1.buf[i];}break;
//					case 3:	for(int i=0;i<16;i++){motor_receive_go8[3].buf[i]=motor_buffer1.buf[i];}break;
					case 4: for(int i=0;i<16;i++){motor_receive_go8[4].buf[i]=motor_buffer1.buf[i];}break;
					case 5:	for(int i=0;i<16;i++){motor_receive_go8[5].buf[i]=motor_buffer1.buf[i];}break;
//					case 6: for(int i=0;i<16;i++){motor_receive_go8[6].buf[i]=motor_buffer.buf[i];}break;
//					case 7:	for(int i=0;i<16;i++){motor_receive_go8[7].buf[i]=motor_buffer.buf[i];}break;
					
			}
			if(flag_zero_2==0&&
				motor_receive_go8[0].rec_data.rad !=0)
				//&&motor_receive_go8[1].rec_data.rad !=0
				//&&motor_receive_go8[4].rec_data.rad !=0
				//&&motor_receive_go8[5].rec_data.rad !=0)
//				&&motor_receive_go8[2].rec_data.rad !=0
//				&&motor_receive_go8[3].rec_data.rad !=0
//				&&motor_receive_go8[4].rec_data.rad !=0
//				&&motor_receive_go8[5].rec_data.rad !=0))////
				
			{
				//printf("ok");
				jiao_zero(0);
				jiao_zero(1);
//				jiao_zero(2);
//				jiao_zero(3);
				jiao_zero(4);
				jiao_zero(5);
//				jiao_zero(6);
//				jiao_zero(7);
				flag_zero_2=1;
			}
			
		}
		remote_control_init2();
	}
	
  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断 
}

void USART6_IRQHandler(void)                	//串口5中断服务程序
{
	/* enter interrupt */
	rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断

	u16 data;
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)  //
	{
		USART_ClearITPendingBit(USART6,USART_IT_IDLE); // 清除标志位
		DMA_Cmd(DMA2_Stream1, DISABLE);
		data = USART6->SR;
		data = USART6->DR;  	// 这里必须读一下串口的SR和DR寄存器  这样程序才能正常运行
		DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);
		while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
		//printf("motor%d %d\r\n",motor_buffer.rec_data.rad,motor_buffer.rec_data.rads);
		//printf("crc%d %d\r\n",motor_buffer.rec_data.crc,crc_ccitt(0,motor_buffer.buf,14));
		if(motor_buffer6.rec_data.crc ==crc_ccitt(0,motor_buffer6.buf,14))
        {
			
			//printf("enter");
			switch(motor_buffer6.rec_data.id)
			{
//					case 0: for(int i=0;i<16;i++){motor_receive_go8[0].buf[i]=motor_buffer6.buf[i];}break;
//					case 1:	for(int i=0;i<16;i++){motor_receive_go8[1].buf[i]=motor_buffer6.buf[i];}break;
					case 2: for(int i=0;i<16;i++){motor_receive_go8[2].buf[i]=motor_buffer6.buf[i];}break;
					case 3:	for(int i=0;i<16;i++){motor_receive_go8[3].buf[i]=motor_buffer6.buf[i];}break;
//					case 4: for(int i=0;i<16;i++){motor_receive_go8[4].buf[i]=motor_buffer.buf[i];}break;
//					case 5:	for(int i=0;i<16;i++){motor_receive_go8[5].buf[i]=motor_buffer.buf[i];}break;
					case 6: for(int i=0;i<16;i++){motor_receive_go8[6].buf[i]=motor_buffer6.buf[i];}break;
					case 7:	for(int i=0;i<16;i++){motor_receive_go8[7].buf[i]=motor_buffer6.buf[i];}break;
					
			}
			if(flag_zero_1==0
				&&motor_receive_go8[7].rec_data.rad !=0
				&&motor_receive_go8[6].rec_data.rad !=0
				&&motor_receive_go8[2].rec_data.rad !=0
				&&motor_receive_go8[3].rec_data.rad !=0
				)
//				&&motor_receive_go8[2].rec_data.rad !=0
//				&&motor_receive_go8[3].rec_data.rad !=0
//				&&motor_receive_go8[4].rec_data.rad !=0
//				&&motor_receive_go8[5].rec_data.rad !=0)//)////
				
			{
				//printf("ok");
//				jiao_zero(0);
//				jiao_zero(1);
				jiao_zero(2);
				jiao_zero(3);
//				jiao_zero(4);
//				jiao_zero(5);
				jiao_zero(6);
				jiao_zero(7);
				flag_zero_1=1;
			}
			remote_control_init();
		}
	}
	
  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断 
} 
