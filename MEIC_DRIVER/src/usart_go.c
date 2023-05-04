#include "include.h"

MOTOR_REC_T motor_receive_go8[8];
MOTOR_SEN_T motor_send_go8[8];	//id,工作模式，转矩，速度，位置，刚度系数，阻尼系数
MOTOR_SEN_T motor_SET[8];

//unsigned int strbuf1[20];   
//unsigned int strbuf2[10];
//unsigned int strbuf3[10];   //三个数组用于位运算 
//unsigned int sign=0;        //判断标志位 
unsigned short int const crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
static unsigned short int crc_ccitt_byte(unsigned short int crc, const unsigned char c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
 unsigned short int crc_ccitt(unsigned short int crc, unsigned char *buffer,size_t len)
{
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}

extern int nm1;
extern int nm2;
extern _angle anglenow;
extern _legde leg_d;
extern int ns1;
extern int x;

float nm_jie(int x) //处理传回来的力矩数据
{
	int a=0;
	if(motor_receive_go8[x].rec_data.nm>=32768)
	{
		a=motor_receive_go8[x].rec_data.nm-65535;
	}
	else
	{
		a=motor_receive_go8[x].rec_data.nm;
	}
	return a;
}

int index1[8];
int last_angle[8];

void rad_zhengfu(int leg_1,int leg_2)
{
	if(((leg_1==0&&leg_2==1)||(leg_1==4&&leg_2==5))&&last_angle[leg_2]!=0&&last_angle[leg_1]!=0)
	{
		if(last_angle[leg_2]-anglenow.anglenow[leg_2]>0||last_angle[leg_1]-anglenow.anglenow[leg_1]<0)
		{
			if(last_angle[leg_2]-anglenow.anglenow[leg_2]>0)
			{
				index1[leg_2]=1;
			}
			if(last_angle[leg_1]-anglenow.anglenow[leg_1]<0)
			{
				index1[leg_1]=1;
			}
			
		}
		else if(last_angle[leg_2]-anglenow.anglenow[leg_2]<0||last_angle[leg_1]-anglenow.anglenow[leg_1]>0)
		{
			if(last_angle[leg_2]-anglenow.anglenow[leg_2]<0)
			{
				index1[leg_2]=-1;
			}
			if(last_angle[leg_1]-anglenow.anglenow[leg_1]>0)
			{
				index1[leg_1]=-1;
			}
		}
	}
	if(((leg_1==2&&leg_2==3)||(leg_1==6&&leg_2==7))&&last_angle[leg_2]!=0&&last_angle[leg_1]!=0)
	{
		if(last_angle[leg_2]-anglenow.anglenow[leg_2]<0||last_angle[leg_1]-anglenow.anglenow[leg_1]>0)
		{
			
			if(last_angle[leg_2]-anglenow.anglenow[leg_2]<0)
			{
				index1[leg_2]=1;
			}
			if(last_angle[leg_1]-anglenow.anglenow[leg_1]>0)
			{
				index1[leg_1]=1;
			}
		}
		else if(last_angle[leg_2]-anglenow.anglenow[leg_2]>0||last_angle[leg_1]-anglenow.anglenow[leg_1]<0)
		{
			if(last_angle[leg_2]-anglenow.anglenow[leg_2]>0)
			{
				index1[leg_2]=-1;
			}
			if(last_angle[leg_1]-anglenow.anglenow[leg_1]<0)
			{
				index1[leg_1]=-1;
			}
		}
	}
	last_angle[leg_2]=anglenow.anglenow[leg_2];
	last_angle[leg_1]=anglenow.anglenow[leg_1];
}

void rad_zhengfu_deng(int leg_1,int leg_2,_legde *legdel)
{
	if(((leg_1==0&&leg_2==1)||(leg_1==4&&leg_2==5))&&legdel->last_angle_now_deng[leg_2]!=0&&legdel->last_angle_now_deng[leg_1]!=0)
	{
		if(legdel->last_angle_now_deng[leg_2]-legdel->angle_now_deng[leg_2]>0||legdel->last_angle_now_deng[leg_1]-legdel->angle_now_deng[leg_1]<0)
		{
			if(legdel->last_angle_now_deng[leg_2]-legdel->angle_now_deng[leg_2]>0)
			{
				index1[leg_2]=1;
			}
			if(legdel->last_angle_now_deng[leg_1]-legdel->angle_now_deng[leg_1]<0)
			{
				index1[leg_1]=1;
			}
			
		}
		else if(legdel->last_angle_now_deng[leg_2]-legdel->angle_now_deng[leg_2]<0||legdel->last_angle_now_deng[leg_1]-legdel->angle_now_deng[leg_1]>0)
		{
			if(legdel->last_angle_now_deng[leg_2]-legdel->angle_now_deng[leg_2]<0)
			{
				index1[leg_2]=-1;
			}
			if(legdel->last_angle_now_deng[leg_1]-legdel->angle_now_deng[leg_1]>0)
			{
				index1[leg_1]=-1;
			}
		}
	}
	if(((leg_1==2&&leg_2==3)||(leg_1==6&&leg_2==7))&&legdel->last_angle_now_deng[leg_2]!=0&&legdel->last_angle_now_deng[leg_1]!=0)
	{
		if(legdel->last_angle_now_deng[leg_2]-legdel->angle_now_deng[leg_2]<0||legdel->last_angle_now_deng[leg_1]-legdel->angle_now_deng[leg_1]>0)
		{
			
			if(legdel->last_angle_now_deng[leg_2]-legdel->angle_now_deng[leg_2]<0)
			{
				index1[leg_2]=1;
			}
			if(legdel->last_angle_now_deng[leg_1]-legdel->angle_now_deng[leg_1]>0)
			{
				index1[leg_1]=1;
			}
		}
		else if(legdel->last_angle_now_deng[leg_2]-legdel->angle_now_deng[leg_2]>0||legdel->last_angle_now_deng[leg_1]-legdel->angle_now_deng[leg_1]<0)
		{
			if(legdel->last_angle_now_deng[leg_2]-legdel->angle_now_deng[leg_2]>0)
			{
				index1[leg_2]=-1;
			}
			if(legdel->last_angle_now_deng[leg_1]-legdel->angle_now_deng[leg_1]<0)
			{
				index1[leg_1]=-1;
			}
		}
	}
	legdel->last_angle_now_deng[leg_2]=legdel->angle_now_deng[leg_2];
	legdel->last_angle_now_deng[leg_1]=legdel->angle_now_deng[leg_1];
}

int a_init[8];

void jiao_zero(int n)//电机零点初始化
{
	a_init[n]=motor_receive_go8[n].rec_data.rad;
}

void motor_init(MOTOR_SEN_T *motorset)
{
	motorset->sen_data.nm=0;
	motorset->sen_data.rads =0;
	motorset->sen_data.rad=0;
	motorset->sen_data.kp =0;
	motorset->sen_data.ks =0;
}
void motor_set(MOTOR_SEN_T *motorset,int nm,int rads,int rad,int kp,int ks)
{
	motorset->sen_data.nm=nm;
	motorset->sen_data.rads =rads;
	motorset->sen_data.rad=rad;
	motorset->sen_data.kp =kp;
	motorset->sen_data.ks =ks;
}

void motor_send(int leg_1,MOTOR_SEN_T *motorset)
{
	motor_send_go8[leg_1].sen_data.head=0xeefe;
	motor_send_go8[leg_1].sen_data.id=leg_1;
	motor_send_go8[leg_1].sen_data.status=1;
	motor_send_go8[leg_1].sen_data.nm=motorset->sen_data.nm;
	motor_send_go8[leg_1].sen_data.rads=motorset->sen_data.rads;   //100;
	motor_send_go8[leg_1].sen_data.rad=motorset->sen_data.rad+a_init[leg_1];//
	motor_send_go8[leg_1].sen_data.kp=motorset->sen_data.kp; //400
	motor_send_go8[leg_1].sen_data.ks=motorset->sen_data.ks;  //100;	
	motor_send_go8[leg_1].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_1].buf,15);
	PDout(5)=1;
	rt_thread_mdelay(1);
	if(leg_1==2||leg_1==3||leg_1==6||leg_1==7)
	{
		PDout(5)=1;
		PDout(5)=1;
		PDout(5)=1;
		PDout(5)=1;
		PDout(5)=1;
		fwrite(motor_send_go8[leg_1].buf,1,17,f_u6);
		while((USART6->SR&0X40)==0);
		PDout(5)=0;
	}
	else if(leg_1==0||leg_1==1||leg_1==4||leg_1==5)
	{
		PDout(5)=1;
		PDout(5)=1;
		PDout(5)=1;
		PDout(5)=1;
		PDout(5)=1;
		fwrite(motor_send_go8[leg_1].buf,1,17,f_u1);
		while((USART1->SR&0X40)==0);
		PDout(5)=0;
	}
	
	
	rt_thread_mdelay(1);
	
}

extern MOTOR_SEN_T motor1[8];

void motor_encoder(void)
{
	motor_set(&motor1[0],0,0,0,0,0);// nm, rads, rad, kp, ks
	motor_send(0,&motor1[0]);
	motor_set(&motor1[1],0,0,0,0,0);
	motor_send(1,&motor1[1]);
	motor_set(&motor1[2],0,0,0,0,0);// nm, rads, rad, kp, ks
	motor_send(2,&motor1[2]);
	motor_set(&motor1[3],0,0,0,0,0);// nm, rads, rad, kp, ks
	motor_send(3,&motor1[3]);
	motor_set(&motor1[4],0,0,0,0,0);// nm, rads, rad, kp, ks
	motor_send(4,&motor1[4]);
	motor_set(&motor1[5],0,0,0,0,0);// nm, rads, rad, kp, ks
	motor_send(5,&motor1[5]);
	motor_set(&motor1[6],0,0,0,0,0);// nm, rads, rad, kp, ks
	motor_send(6,&motor1[6]);
	motor_set(&motor1[7],0,0,0,0,0);
	motor_send(7,&motor1[7]);
}

void motor_zhanli(void)
{
	motor_set(&motor1[0],0,radsmotor*5,start_zheng,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(0,&motor1[0]);
	motor_set(&motor1[1],0,-radsmotor*5,-start_zheng,kpmotor_li,ksmotor_li);
	motor_send(1,&motor1[1]);
	motor_set(&motor1[2],0,-radsmotor*5,-start_zheng,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(2,&motor1[2]);
	motor_set(&motor1[3],0,radsmotor*5,start_zheng,kpmotor_li,ksmotor_li);
	motor_send(3,&motor1[3]);
	motor_set(&motor1[4],0,radsmotor*5,start_zheng,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(4,&motor1[4]);
	motor_set(&motor1[5],0,-radsmotor*5,-start_zheng,kpmotor_li,ksmotor_li);
	motor_send(5,&motor1[5]);
	motor_set(&motor1[6],0,-radsmotor*5,-start_zheng,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(6,&motor1[6]);
	motor_set(&motor1[7],0,radsmotor*5,start_zheng,kpmotor_li,ksmotor_li);
	motor_send(7,&motor1[7]);
	rt_thread_mdelay(3000);
	motor_set(&motor1[0],0,radsmotor*2,start_zheng+start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(0,&motor1[0]);
	motor_set(&motor1[1],0,radsmotor*2,-start_zheng+start_ping2,kpmotor_li,ksmotor_li);
	motor_send(1,&motor1[1]);
	motor_set(&motor1[2],0,-radsmotor*2,-start_zheng-start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(2,&motor1[2]);
	motor_set(&motor1[3],0,-radsmotor*2,start_zheng-start_ping2,kpmotor_li,ksmotor_li);
	motor_send(3,&motor1[3]);
	motor_set(&motor1[4],0,radsmotor*2,start_zheng+start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(4,&motor1[4]);
	motor_set(&motor1[5],0,radsmotor*2,-start_zheng+start_ping2,kpmotor_li,ksmotor_li);
	motor_send(5,&motor1[5]);
	motor_set(&motor1[6],0,-radsmotor*2,-start_zheng-start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(6,&motor1[6]);
	motor_set(&motor1[7],0,-radsmotor*2,start_zheng-start_ping2,kpmotor_li,ksmotor_li);
	motor_send(7,&motor1[7]);
	rt_thread_mdelay(3000);
	motor_set(&motor1[0],0,radsmotor*2,mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588+start_zheng+start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(0,&motor1[0]);
	motor_set(&motor1[1],0,radsmotor*2,(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588-start_zheng+start_ping2,kpmotor_li,ksmotor_li);
	motor_send(1,&motor1[1]);
	motor_set(&motor1[2],0,-radsmotor*2,-mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588-start_zheng-start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(2,&motor1[2]);
	motor_set(&motor1[3],0,-radsmotor*2,-(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588+start_zheng-start_ping2,kpmotor_li,ksmotor_li);
	motor_send(3,&motor1[3]);
	motor_set(&motor1[4],0,radsmotor*2,mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588+start_zheng+start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(4,&motor1[4]);
	motor_set(&motor1[5],0,radsmotor*2,(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588-start_zheng+start_ping2,kpmotor_li,ksmotor_li);
	motor_send(5,&motor1[5]);
	motor_set(&motor1[6],0,-radsmotor*2,-mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588-start_zheng-start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(6,&motor1[6]);
	motor_set(&motor1[7],0,-radsmotor*2,-(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588+start_zheng-start_ping2,kpmotor_li,ksmotor_li);
	motor_send(7,&motor1[7]);
}

void motor_pingzhan(void)
{
	motor_set(&motor1[0],80,radsmotor*2,mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588+start_zheng+start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(0,&motor1[0]);
	motor_set(&motor1[1],80,radsmotor*2,(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588-start_zheng+start_ping2,kpmotor_li,ksmotor_li);
	motor_send(1,&motor1[1]);
	motor_set(&motor1[2],-80,-radsmotor*2,-mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588-start_zheng-start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(2,&motor1[2]);
	motor_set(&motor1[3],-80,-radsmotor*2,-(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588+start_zheng-start_ping2,kpmotor_li,ksmotor_li);
	motor_send(3,&motor1[3]);
	motor_set(&motor1[4],80,radsmotor*2,mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588+start_zheng+start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(4,&motor1[4]);
	motor_set(&motor1[5],80,radsmotor*2,(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588-start_zheng+start_ping2,kpmotor_li,ksmotor_li);
	motor_send(5,&motor1[5]);
	motor_set(&motor1[6],-80,-radsmotor*2,-mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588-start_zheng-start_ping1,kpmotor_li,ksmotor_li);// nm, rads, rad, kp, ks
	motor_send(6,&motor1[6]);
	motor_set(&motor1[7],-80,-radsmotor*2,-(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588+start_zheng-start_ping2,kpmotor_li,ksmotor_li);
	motor_send(7,&motor1[7]);
}

void motor_danamic(void)
{
	motor_set(&motor1[0],0,index1[0]*radsmotor*5,anglenow.anglenow[0]*588+start_zheng+start_ping1,kpmotor,ksmotor);// nm, rads, rad, kp, ks
	motor_send(0,&motor1[0]);
	motor_set(&motor1[1],0,index1[1]*radsmotor*5,(180-anglenow.anglenow[1])*588-start_zheng+start_ping2,kpmotor,ksmotor);
	motor_send(1,&motor1[1]);
	motor_set(&motor1[2],0,index1[2]*radsmotor*5,-anglenow.anglenow[2]*588-start_zheng-start_ping1,kpmotor,ksmotor);// nm, rads, rad, kp, ks
	motor_send(2,&motor1[2]);
	motor_set(&motor1[3],0,index1[3]*radsmotor*5,-(180-anglenow.anglenow[3])*588+start_zheng-start_ping2,kpmotor,ksmotor);
	motor_send(3,&motor1[3]);
	motor_set(&motor1[4],0,index1[4]*radsmotor*5,anglenow.anglenow[4]*588+start_zheng+start_ping1,kpmotor,ksmotor);// nm, rads, rad, kp, ks
	motor_send(4,&motor1[4]);
	motor_set(&motor1[5],0,index1[5]*radsmotor*5,(180-anglenow.anglenow[5])*588-start_zheng+start_ping2,kpmotor,ksmotor);
	motor_send(5,&motor1[5]);
	motor_set(&motor1[6],0,index1[6]*radsmotor*5,-anglenow.anglenow[6]*588-start_zheng-start_ping1,kpmotor,ksmotor);// nm, rads, rad, kp, ks ,
	motor_send(6,&motor1[6]);
	motor_set(&motor1[7],0,index1[7]*radsmotor*5,-(180-anglenow.anglenow[7])*588+start_zheng-start_ping2,kpmotor,ksmotor);
	motor_send(7,&motor1[7]);
}

void motor_danamic_deng(void)
{
	motor_set(&motor1[0],index1[0]*80,index1[0]*radsmotor*2,leg_d.angle_now_deng[0]*588+start_zheng+start_ping1,kpmotor,ksmotor);// nm, rads, rad, kp, ks
	motor_set(&motor1[1],index1[1]*80,index1[1]*radsmotor*2,(180-leg_d.angle_now_deng[1])*588-start_zheng+start_ping2,kpmotor,ksmotor);
	motor_set(&motor1[2],index1[2]*(-80),index1[2]*radsmotor*2,-leg_d.angle_now_deng[2]*588-start_zheng-start_ping1,kpmotor,ksmotor);// nm, rads, rad, kp, ks
	motor_set(&motor1[3],index1[3]*(-80),index1[3]*radsmotor*2,-(180-leg_d.angle_now_deng[3])*588+start_zheng-start_ping2,kpmotor,ksmotor);
	motor_set(&motor1[4],index1[4]*80,index1[4]*radsmotor*2,leg_d.angle_now_deng[4]*588+start_zheng+start_ping1,kpmotor,ksmotor);// nm, rads, rad, kp, ks
	motor_set(&motor1[5],index1[5]*80,index1[5]*radsmotor*2,(180-leg_d.angle_now_deng[5])*588-start_zheng+start_ping2,kpmotor,ksmotor);
	motor_set(&motor1[6],index1[6]*(-80),index1[6]*radsmotor*2,-leg_d.angle_now_deng[6]*588-start_zheng-start_ping1,kpmotor,ksmotor);// nm, rads, rad, kp, ks ,
	motor_set(&motor1[7],index1[7]*(-80),index1[7]*radsmotor*2,-(180-leg_d.angle_now_deng[7])*588+start_zheng-start_ping2,kpmotor,ksmotor);
	motor_send(2,&motor1[2]);
	motor_send(3,&motor1[3]);
	motor_send(0,&motor1[0]);
	motor_send(1,&motor1[1]);
	motor_send(6,&motor1[6]);
	motor_send(7,&motor1[7]);
	motor_send(4,&motor1[4]);
	motor_send(5,&motor1[5]);
	
}



//void motor_fuwei(int leg_1)
//{
//	if(leg_1 ==0)
//	{
//		motor_send_go8[leg_1].sen_data.head=0xeefe;
//		motor_send_go8[leg_1].sen_data.id=leg_1;
//		motor_send_go8[leg_1].sen_data.status=1;
//		motor_send_go8[leg_1].sen_data.nm=0;
//		motor_send_go8[leg_1].sen_data.rads=-100;   //100;
//		motor_send_go8[leg_1].sen_data.rad=9714;
//		motor_send_go8[leg_1].sen_data.kp=400; //
//		motor_send_go8[leg_1].sen_data.ks=100;  //;	
//		motor_send_go8[leg_1].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_1].buf,15);
//		PDout(5)=1;
//		rt_thread_mdelay(1);
//		fwrite(motor_send_go8[leg_1].buf,1,17,f_u6);
//		while((USART6->SR&0X40)==0);
//		PDout(5)=0;
//		rt_thread_mdelay(1);
//	}
//	else if(leg_1 ==1)
//	{
//		motor_send_go8[leg_1].sen_data.head=0xeefe;
//		motor_send_go8[leg_1].sen_data.id=leg_1;
//		motor_send_go8[leg_1].sen_data.status=1;
//		motor_send_go8[leg_1].sen_data.nm=0;
//		motor_send_go8[leg_1].sen_data.rads=-100;   //100;
//		motor_send_go8[leg_1].sen_data.rad=6396;
//		motor_send_go8[leg_1].sen_data.kp=400; //
//		motor_send_go8[leg_1].sen_data.ks=100;  //;	
//		motor_send_go8[leg_1].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_1].buf,15);
//		PDout(5)=1;
//		rt_thread_mdelay(1);
//		fwrite(motor_send_go8[leg_1].buf,1,17,f_u6);
//		while((USART6->SR&0X40)==0);
//		PDout(5)=0;
//		rt_thread_mdelay(1);
//	}
//	else if(leg_1 ==6)
//	{
//		motor_send_go8[leg_1].sen_data.head=0xeefe;
//		motor_send_go8[leg_1].sen_data.id=leg_1;
//		motor_send_go8[leg_1].sen_data.status=1;
//		motor_send_go8[leg_1].sen_data.nm=0;
//		motor_send_go8[leg_1].sen_data.rads=100;   //100;
//		motor_send_go8[leg_1].sen_data.rad=16742;
//		motor_send_go8[leg_1].sen_data.kp=400; //
//		motor_send_go8[leg_1].sen_data.ks=100;  //;	
//		motor_send_go8[leg_1].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_1].buf,15);
//		PDout(5)=1;
//		rt_thread_mdelay(1);
//		fwrite(motor_send_go8[leg_1].buf,1,17,f_u6);
//		while((USART6->SR&0X40)==0);
//		PDout(5)=0;
//		rt_thread_mdelay(1);
//	}
//	else if(leg_1 ==7)
//	{
//		motor_send_go8[leg_1].sen_data.head=0xeefe;
//		motor_send_go8[leg_1].sen_data.id=leg_1;
//		motor_send_go8[leg_1].sen_data.status=1;
//		motor_send_go8[leg_1].sen_data.nm=0;
//		motor_send_go8[leg_1].sen_data.rads=100;   //100;
//		motor_send_go8[leg_1].sen_data.rad=6359;
//		motor_send_go8[leg_1].sen_data.kp=400; //
//		motor_send_go8[leg_1].sen_data.ks=100;  //;	
//		motor_send_go8[leg_1].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_1].buf,15);
//		PDout(5)=1;
//		rt_thread_mdelay(1);
//		fwrite(motor_send_go8[leg_1].buf,1,17,f_u6);
//		while((USART6->SR&0X40)==0);
//		PDout(5)=0;
//		rt_thread_mdelay(1);
//	}
//}

//void send_motor_tree()

//void send_motoryvtree_stand(int leg_1,int leg_2,MOTOR_SEN_T *motorset,MOTOR_SEN_T *motorset1) //让狗站起来
//{
//	
//	
//			motor_send_go8[leg_1].sen_data.head=0xeefe;
//			motor_send_go8[leg_1].sen_data.id=leg_1;
//			motor_send_go8[leg_1].sen_data.status=1;
//			motor_send_go8[leg_1].sen_data.nm=nm1-motorset->sen_data.nm;
//			motor_send_go8[leg_1].sen_data.rads=index1[leg_1]*motorset->sen_data.rads;   //100;
//			motor_send_go8[leg_1].sen_data.rad=-(mathematicalmodelparallel_contrary_corner1(0.05,27.94,13.39,26,0))*588+a_init[leg_1];
//			motor_send_go8[leg_1].sen_data.kp=motorset->sen_data.kp; //400
//			motor_send_go8[leg_1].sen_data.ks=motorset->sen_data.ks;  //100;	
//			motor_send_go8[leg_1].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_1].buf,15);
////			
//			motor_send_go8[leg_2].sen_data.head=0xeefe;
//			motor_send_go8[leg_2].sen_data.id=leg_2;
//			motor_send_go8[leg_2].sen_data.status=1;
//			motor_send_go8[leg_2].sen_data.nm=nm2-30;
//			motor_send_go8[leg_2].sen_data.rads=index1[leg_2]*100;
//			motor_send_go8[leg_2].sen_data.rad=-((180-mathematicalmodelparallel_contrary_corner2(0.05,27.94,13.39,26,0))*588)+a_init[leg_2];
//			motor_send_go8[leg_2].sen_data.kp=400;
//			motor_send_go8[leg_2].sen_data.ks=100;	
//			motor_send_go8[leg_2].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_2].buf,15);
//			
//			rt_thread_mdelay(1);
//			fwrite(motor_send_go8[leg_1].buf,1,17,f_u6);
//			rt_thread_mdelay(1);
//			fwrite(motor_send_go8[leg_2].buf,1,17,f_u6);
//			while((USART6->SR&0X40)==0);
//}

//void send_motoryvtree_walk(int leg_1,int leg_2,MOTOR_SEN_T *motorset,MOTOR_SEN_T *motorset1) //让狗走起来
//{
//			motor_send_go8[leg_1].sen_data.head=0xeefe;
//			motor_send_go8[leg_1].sen_data.id=leg_1;
//			motor_send_go8[leg_1].sen_data.status=1;
//			motor_send_go8[leg_1].sen_data.nm=nm1-30;
//			motor_send_go8[leg_1].sen_data.rads=index1[leg_1]*100;
//			motor_send_go8[leg_1].sen_data.rad=-(anglenow.anglenow[leg_1])*588+a_init[leg_1];
//			motor_send_go8[leg_1].sen_data.kp=400;
//			motor_send_go8[leg_1].sen_data.ks=100;	
//			motor_send_go8[leg_1].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_1].buf,15);
////			
//			motor_send_go8[leg_2].sen_data.head=0xeefe;
//			motor_send_go8[leg_2].sen_data.id=leg_2;
//			motor_send_go8[leg_2].sen_data.status=1;
//			motor_send_go8[leg_2].sen_data.nm=nm2-30;
//			motor_send_go8[leg_2].sen_data.rads=index1[leg_2]*100;
//			motor_send_go8[leg_2].sen_data.rad=-((180-anglenow.anglenow[leg_2])*588)+a_init[leg_2];
//			motor_send_go8[leg_2].sen_data.kp=400;
//			motor_send_go8[leg_2].sen_data.ks=100;	
//			motor_send_go8[leg_2].sen_data.crc= crc_ccitt(0,motor_send_go8[leg_2].buf,15);
//			
//			rt_thread_mdelay(1);
//			fwrite(motor_send_go8[leg_1].buf,1,17,f_u6);
//			rt_thread_mdelay(1);
//			fwrite(motor_send_go8[leg_2].buf,1,17,f_u6);
//			while((USART6->SR&0X40)==0);
//}

//以下为废弃协议代码

//void usart_send_go8(MOTOR_SEN_T *motorsend)    //通过位运算处理传输的数据
//{
//	motorsend->buf[0]=0xFE;
//	motorsend->buf[1]=0xEE;
//	motorsend->sen_data.status=motorsend ->sen_data.status<<4;
//	motorsend->buf[2]=/*motorsend ->sen_data.id|*/motorsend ->sen_data.status;	//id和状态合并发送
//	motorsend->buf[3]=motorsend ->sen_data.nm&0xff;
//	motorsend->buf[4]=(motorsend ->sen_data.nm>>8)&0xff;			//先传低位，后传高位 期望转矩			
//	motorsend->buf[5]=motorsend ->sen_data.rads&0xff;
//	motorsend->buf[6]=(motorsend ->sen_data.rads>>8)&0xff;		//期望转速
//	motorsend->buf[7]=motorsend ->sen_data.rad&0xff;
//	motorsend->buf[8]=(motorsend ->sen_data.rad>>8)&0xff;
//	motorsend->buf[9]=(motorsend ->sen_data.rad>>16)&0xff;
//	motorsend->buf[10]=(motorsend ->sen_data.rad>>24)&0xff;		//期望位置
//	motorsend->buf[11]=motorsend ->sen_data.kp&0xff;
//	motorsend->buf[12]=(motorsend ->sen_data.kp>>8)&0xff;					//刚度系数
//	motorsend->buf[13]=motorsend ->sen_data.ks&0xff;
//	motorsend->buf[14]=(motorsend ->sen_data.ks>>8)&0xff;					//阻尼系数
//	motorsend->buf[15]=crc_ccitt(0x1021,motorsend->buf,120)&0xff;		//crc校验，前八位 
//	motorsend->buf[16]=crc_ccitt(0x1021,motorsend->buf,120)>>8;				//crc校验，后八位 
//}

//void usart_receive_go8(MOTOR_REC_T *motorsend)
//{
//	motorsend->rec_data.id=strbuf3[0];
//	motorsend->rec_data.status=strbuf3[1];
//	motorsend->rec_data.nm=strbuf3[2]/256 ;
//	motorsend->rec_data.rads=strbuf3[3]/256*2*PI ;
//	motorsend->rec_data.rad=strbuf3[4]/32768*2*PI ;
//	motorsend->rec_data.temp=strbuf1[6];
//	motorsend->rec_data.merror=strbuf3[5];
//	motorsend->rec_data.force=strbuf3[7];
//	motorsend->rec_data.crc=strbuf2[5];
//	motorsend->buf[0]=0xFD;
//	motorsend->buf[1]=0xEE;
//	motorsend->buf[2]=motorsend->rec_data.status<<4;
//	motorsend->buf[3]= motorsend->rec_data.id| motorsend->buf[2];
//	motorsend->buf[4]=motorsend->rec_data.nm&0xff;
//	motorsend->buf[5]=(motorsend->rec_data.nm>>8)&0xff;
//	motorsend->buf[6]= motorsend->rec_data.rads&0xff;
//	motorsend->buf[7]=( motorsend->rec_data.rads>>8)&0xff;
//	motorsend->buf[8]=motorsend->rec_data.rad&0xff;
//	motorsend->buf[9]=(motorsend->rec_data.rad>>8)&0xff;
//	motorsend->buf[10]=(motorsend->rec_data.rad>>16)&0xff;
//	motorsend->buf[11]=(motorsend->rec_data.rad>>24)&0xff;
//	motorsend->buf[12]=motorsend->rec_data.temp;
//	motorsend->buf[13]=motorsend->rec_data.merror| (motorsend->rec_data.force&0x1f);
//	motorsend->buf[14]=motorsend->rec_data.force>>5;
//	motorsend->crcbuf=crc_ccitt(0x1021,motorsend->buf,120);
//}

//void clear_zero(void)
//{
//	motor_receive_go8.rec_data.id=0;
//	motor_receive_go8.rec_data.status=0;
//	motor_receive_go8.rec_data.nm=0;
//	motor_receive_go8.rec_data.rads=0;
//	motor_receive_go8.rec_data.rad=0;
//	motor_receive_go8.rec_data.temp=0;
//	motor_receive_go8.rec_data.merror=0;
//	motor_receive_go8.rec_data.force=0;
//}

//void usart_go8(unsigned char res)
//{

//	switch(sign)
//	{
//		case 0: motor_receive_go8.buf[0]=res;sign=1;break;
//		case 1: motor_receive_go8.buf[1]=res;sign=2;break;       //串口中断一次已接收了8位即一个字节，通过标志位判断包头
//		case 2: if(motor_receive_go8.rec_data.head==0xeefd){sign=3;}
//				    else {sign=0;}break;
//		case 3: motor_receive_go8.buf[2]=res;sign=4;break ;
//		case 4:	motor_receive_go8.buf[3]=res;sign=5;break ;   //实际关节输出转矩，为2个字节，串口中断两次，接收的数据只须移位转整形再相加 
//		case 5: motor_receive_go8.buf[4]=res;sign=6;break;
//		case 6: motor_receive_go8.buf[5]=res;sign=7;break;   //实际关节输出速度，与转矩同理 
//		case 7: motor_receive_go8.buf[6]=res;sign=8;break;
//		case 8: motor_receive_go8.buf[7]=res;sign=9;break;
//		case 9: motor_receive_go8.buf[8]=res;sign=10;break;
//		case 10: motor_receive_go8.buf[9]=res;sign=11;break;	  	//实际关节输出位置，为四个字节，与转矩同理，按理来讲32位或64位处理器int型为4个字节，为了保险选择long型 		 
//		case 11: motor_receive_go8.buf[10]=res; sign=12;break;            //电机温度，为一个字节 	 
//		case 12: motor_receive_go8.buf[11]=res;sign=13;break;				//后五位为足端力保存下来 		 
//		case 13: motor_receive_go8.buf[12]=res;sign=14;break;   //足端力，为12位，左移六位，把前五位空出来，再加在一起 	 
//		case 14: motor_receive_go8.buf[13]=res;sign=15;break;	 	
//		case 15: motor_receive_go8.buf[14]=res;sign=16;break;
//		case 16: motor_receive_go8.buf[15]=res;usart_receive_go8(&motor_receive_go8);
//				     sign=0;break;
//	}
//}
