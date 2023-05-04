#ifndef __DRIVER_USART_H
#define __DRIVER_USART_H

#define EN_PRINTF 1   /*是否开启printf支持*/

#include "stm32f4xx.h"
#include "stdio.h"
#include <rtthread.h>
#define f_u1 (FILE*)USART1
#define f_u2 (FILE*)USART2
#define f_u3 (FILE*)USART3
#define f_u6 (FILE*)USART6
#define f_u4 (FILE*)UART4
#define f_u5 (FILE*)UART5
extern u8 Res;
extern int16_t slave_X;               //图像X坐标 
extern int16_t slave_Y;               //图像Y坐标 
extern int16_t slave_ERR;             //转角值 

extern char cmdopen[5];
extern char cmdzero[5];
extern float yaw_error;

void rodicmd(void);
void USART6_Config(void);
void USART2_Config(void);
void get_slave_data(u8 data) ;
void data_analysis(u8 *line) ;

void CopeSerial2Data(unsigned char ucData);				
void Get_acc(float *accx,float *accy,float *accz);
void Get_gyro(float *gyrox,float *gyroy,float *gyroz);
void Get_angle(float *roll,float *pitch,float *yaw);
void jy61_init(void);
void Get_pitch(float *pitch);
void Get_yaw(float *yaw);
void sendcmd(char cmd[]);
void Get_yaw_error(float *yaw);
void Dispose_JY61_yaw(float *yaw,float yaw_error);
#endif
