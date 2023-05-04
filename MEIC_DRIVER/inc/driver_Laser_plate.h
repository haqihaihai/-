#ifndef __DRIVER_LASER_PLATE_H
#define __DRIVER_LASER_PLATE_H

#include "stm32f4xx.h"



typedef 	union date_change{						//数据//可以升级发浮点型
		float _fl_date;
		u8 	_u8[4];
}UartOnlinDate;





void Laser_decode(uint8_t Res);


extern float laser_Res[4];

#define UART5_MAX_RECV_LEN		200					//最大接收缓存字节数
#define UART5_MAX_SEND_LEN		200					//最大发送缓存字节数
#define UART5_RX_EN 			1					//0,不接收;1,接收.

extern u8  UART5_RX_BUF[UART5_MAX_RECV_LEN]; 		//接收缓冲,最大USART2_MAX_RECV_LEN字节
extern u8  UART5_TX_BUF[UART5_MAX_SEND_LEN]; 		//发送缓冲,最大USART2_MAX_SEND_LEN字节
extern u16 UART5_RX_STA;   						//接收数据状态

void uart5_init(u32 bound);		
void u5_printf(char* fmt,...);


#endif


