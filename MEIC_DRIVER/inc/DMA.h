#ifndef __DMA_H
#define	__DMA_H	   
#include "sys.h"
#include "BSP_ALL.h"

#define EN_USART1_RX 			1	
#define USART1_REC_LEN  			200  	//定义最大接收字节数 200

#define EN_UART5_RX 			1	
#define UART5_REC_LEN  			200  	//定义最大接收字节数 200

typedef struct 
{ 
	 int16_t ch0; 
	 int16_t ch1; 
	 int16_t ch2; 
	 int16_t ch3; 
	 uint8_t s1; 
	 uint8_t s2; 
}rc_t; 

void pa_init(void);
void UART5_DMA_INIT(void);
void USART1_DMA_Init(void);
void rc_init(void);
#endif

