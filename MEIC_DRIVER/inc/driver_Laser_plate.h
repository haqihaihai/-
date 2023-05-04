#ifndef __DRIVER_LASER_PLATE_H
#define __DRIVER_LASER_PLATE_H

#include "stm32f4xx.h"



typedef 	union date_change{						//����//����������������
		float _fl_date;
		u8 	_u8[4];
}UartOnlinDate;





void Laser_decode(uint8_t Res);


extern float laser_Res[4];

#define UART5_MAX_RECV_LEN		200					//�����ջ����ֽ���
#define UART5_MAX_SEND_LEN		200					//����ͻ����ֽ���
#define UART5_RX_EN 			1					//0,������;1,����.

extern u8  UART5_RX_BUF[UART5_MAX_RECV_LEN]; 		//���ջ���,���USART2_MAX_RECV_LEN�ֽ�
extern u8  UART5_TX_BUF[UART5_MAX_SEND_LEN]; 		//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
extern u16 UART5_RX_STA;   						//��������״̬

void uart5_init(u32 bound);		
void u5_printf(char* fmt,...);


#endif


