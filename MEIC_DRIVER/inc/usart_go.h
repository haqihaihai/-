#ifndef __USART_GO_H
#define __USART_GO_H
#include "stddef.h"
#include "foot_trajectory.h"
//#define PI acos(-1)


#pragma pack (1)
typedef union MOTOR_REC{
	struct MOTOR_REC_S{
		long head:16;
		long id:4;
		long status:3;
		long :1;
		long nm:16;
		long rads:16;
		long rad:32;
		long temp:8;
		long merror:3;
		long force :12;
		long :1;
		unsigned long crc:16;
	}rec_data;
	unsigned char buf[16];
	unsigned short crcbuf;
}MOTOR_REC_T;
typedef  union MOTOR_SEN{
	struct MOTOR_SEN_S{
		long head:16;
		long id:4;
		long status:3;
		long :1;
		long nm:16;
		long rads:16;
		long rad:32;
		long kp:16;
		long ks:16;
		unsigned long crc:16;
	}sen_data;
	unsigned char buf[17];
}MOTOR_SEN_T;
#pragma pack (4)
//static unsigned short int crc_ccitt_byte(unsigned short int crc, const unsigned char c);


/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
unsigned short int crc_ccitt(unsigned short int crc, unsigned char *buffer,size_t len);
void motor_init(MOTOR_SEN_T *motorset);
void rad_zhengfu(int leg_1,int leg_2);
float nm_jie(int x);
void jiao_zero(int n);
void motor_encoder(void);
void motor_zhanli(void);
void motor_pingzhan(void);
void motor_danamic(void);
void motor_set(MOTOR_SEN_T *motorset,int nm,int rads,int rad,int kp,int ks);
void motor_send(int leg_1,MOTOR_SEN_T *motorset);

void rad_zhengfu_deng(int leg_1,int leg_2,_legde *legdel);
void motor_danamic_deng(void);
//void motor_fuwei(int leg_1);
//void send_motoryvtree_stand(int leg_1,int leg_2,MOTOR_SEN_T *motorset,MOTOR_SEN_T *motorset1);
//void send_motoryvtree_walk(int leg_1,int leg_2,MOTOR_SEN_T *motorset,MOTOR_SEN_T *motorset1);

//·ÏÆúº¯Êý
//void usart_go8(unsigned char res);
//void usart_send_go8(MOTOR_SEN_T *motorsend);
//void usart_receive_go8(MOTOR_REC_T *motorsend);
//void clear_zero(void);


#endif
