#include "include.h"

extern struct rt_timer timer1,timer2,timer3,timer4;
int flag8=0;
extern int ns[4];
extern rc_t rc_control;
extern MOTOR_SEN_T motor_send_go8[8];
extern MOTOR_REC_T motor_receive_go8[8];
//extern struct SAcc 		stcAcc;
//extern struct SGyro 	stcGyro;
//extern struct SAngle 	stcAngle; 
//extern int nm1;
//extern int nm2;
//extern int a,b;
extern _angle anglenow;
extern int nm1;
extern int nm2;
extern int nm3;
extern int nm4;

//测试结果为一度是588

/*
先让足端轨迹的最高点摆动到y轴mathematicalmodelparallel_contrary_corner1(16.21,28.75,13.39,26,0)
之后站立起来之后进行踏步
设标志位，踏步到摆动周期的一般的时候进行迈步mathematicalmodelparallel_contrary_corner2(16.21,28.75,13.39,26,0)
*/

MOTOR_SEN_T motor1[8];
extern int a_init[8];
extern int flag_zero;
extern int index1[8];
extern _rhy rhy1,rhy2;
extern int flag_zero_1;
extern int flag_zero_2;

int state_motor=0;
int state_motor_cong=0;
int state_buye=0;
int flag_zouyou=0;
_rhy rhy1_1;
_rhy rhy2_2;
_legde leg_d;

void thread_entry1(void* parameter)
{
//	rhy_init(&rhy1_1);
//	rhy_init(&rhy2_2);
//	rhy_init(&rhy1);
//	rhy_init(&rhy2);
	flag_zero=0;
	deng_init(&leg_d);
	//deng_butai_init(&leg_d,0.5,10,15,0.05,27.94,-14.95,15.05);
	deng_butai_init(&leg_d,0.5,10,15,0.0,30.94,-10,10);
	PDout(5)=0;
    while (1)
    {
//			if(flag_zero_1==1&& flag_zero_2==1&&flag_zero==0)
//			{
//				flag_zero=1;
//			}
//			if(flag_zero==2&&rc_control.s2==2)
//			{
//				if(rc_control.s1==1) //前进后退向左向右
//				{
//					if(rc_control.ch1>0)
//					{
//						if(rc_control.ch1>=-50&&rc_control.ch1<=50)
//						{
//							motor_pingzhan();  //站立
//							t1_second=0;
//						}
//						else
//						{
//							//state_buye=30;
//							rhy1_1.phase2=0.5;
//							rhy1_1.duty2=0.5;
//							rhy1_1.H2=15;
//							rhy1_1.T2=50;

//							rhy2_2.phase2=0.5;
//							rhy2_2.duty2=0.5;
//							rhy2_2.H2=15;
//							rhy2_2.T2=50;

//							if(rc_control.ch0<=-10)
//							{
//							
//									rhy2_2.S2=40;
//									rhy1_1.S2=25;
//							}
//							else if(rc_control.ch0>=10)
//							{
////							
//									rhy1_1.S2=40;
//									rhy2_2.S2=25;
//							}
//							else if(rc_control.ch0<=10&&rc_control.ch0>=-10)
//							{
//								rhy1_1.S2=30;
//								rhy2_2.S2=30;
//							}
//							
//							state_motor=tort_qianjin;
//							motor_danamic();
//						}
//					}
//					if(rc_control.ch1<0)
//					{
//						if(rc_control.ch1>=-50&&rc_control.ch1<=50)
//						{
//							motor_pingzhan();  //站立
//							t1_second=0;
//						}
//						else
//						{

//							rhy1_1.phase2=0.5;
//							rhy1_1.duty2=0.5;
//							rhy1_1.H2=15;
//							rhy1_1.T2=50;

//							rhy2_2.phase2=0.5;
//							rhy2_2.duty2=0.5;
//							rhy2_2.H2=15;
//							rhy2_2.T2=50;

//							if(rc_control.ch0<=-10)
//							{
//								
//									rhy2_2.S2=40;
//									rhy1_1.S2=25;
//							}
//							else if(rc_control.ch0>=10)
//							{
//									rhy1_1.S2=40;
//									rhy2_2.S2=25;
//							}
//							else if(rc_control.ch0<=10&&rc_control.ch0>=-10)
//							{
//								rhy1_1.S2=30;
//								rhy2_2.S2=30;
//							}
//							
//							state_motor=tort_houtui; 
//							motor_danamic();
//						}
//					}
//				}
//				else if(rc_control.s1==2) //自旋
//				{
//					rhy1_1.phase2=0.5;
//					rhy1_1.duty2=0.5;
//					rhy1_1.H2=7.5;
//					rhy1_1.S2=12;
//					rhy1_1.T2=50;

//					rhy2_2.phase2=0.5;
//					rhy2_2.duty2=0.5;
//					rhy2_2.H2=7.5;
//					rhy2_2.S2=12;
//					rhy2_2.T2=50;
//					state_motor=zixua;
//					if(rc_control.ch0<=-10)
//					{		
//						state_motor_cong=nixuan;
//					}
//					else if(rc_control.ch0>=10)
//					{
//						state_motor_cong=shunxuan;
//					}
//					else if(rc_control.ch0<=10&&rc_control.ch0>=-10)
//					{
//						motor_pingzhan();  //站立
//					}
//				}
//			}
//			if(rc_control.s1==3&&flag_zero==1&&rc_control.s2==3)
//			{
//				motor_zhanli();  //站立
//				flag_zero=2;
//			}
//			if(rc_control.s1==2&&flag_zero==0&&rc_control.s2==3)//
//			{
//				motor_encoder();    //发送零力拒进行较零
//			}

//灯哥
//			if(flag_zero_1==1&& flag_zero_2==1&&flag_zero==0)
//			{
//				flag_zero=1;
//			}
//			if(flag_zero==2&&rc_control.s2==2)
//			{
//				if(rc_control.s1==1) //前进后退向左向右
//				{
//					if(rc_control.ch1>0)
//					{
//						if(rc_control.ch1>=-50&&rc_control.ch1<=50)
//						{
//							motor_pingzhan();  //站立
//							leg_d.t1_second=0;
//						}
//						else
//						{
//							if(rc_control.ch0<=-50)
//							{
//								set_bugao(&leg_d,0,-15,15);
//								set_bugao(&leg_d,2,-15,15);
//								set_bugao(&leg_d,1,-8,8);
//								set_bugao(&leg_d,3,-8,8);
//							}
//							if(rc_control.ch0>=50)
//							{
//								set_bugao(&leg_d,0,-8,8);
//								set_bugao(&leg_d,2,-8,8);
//								set_bugao(&leg_d,1,-15,15);
//								set_bugao(&leg_d,3,-15,15);
//							}
//							if(rc_control.ch0<=50&&rc_control.ch0>=-50)
//							{
//								set_bugao(&leg_d,0,-10,10);
//								set_bugao(&leg_d,2,-10,10);
//								set_bugao(&leg_d,1,-10,10);
//								set_bugao(&leg_d,3,-10,10);
//							}
//							leg_d.state=tort_qianjin;
//							motor_danamic_deng();
//							
//						}
//					}
//					if(rc_control.ch1<0)
//					{
//						
//						if(rc_control.ch1>=-50&&rc_control.ch1<=50)
//						{
//							motor_pingzhan();  //站立
//							leg_d.t1_second=0;
//						}
//						else
//						{
//							if(rc_control.ch0<=-50)
//							{
//								set_bugao(&leg_d,0,-15,15);
//								set_bugao(&leg_d,2,-15,15);
//								set_bugao(&leg_d,1,-8,8);
//								set_bugao(&leg_d,3,-8,8);
//							}
//							if(rc_control.ch0>=50)
//							{
//								set_bugao(&leg_d,0,-8,8);
//								set_bugao(&leg_d,2,-8,8);
//								set_bugao(&leg_d,1,-15,15);
//								set_bugao(&leg_d,3,-15,15);
//							}
//							if(rc_control.ch0<=50&&rc_control.ch0>=-50)
//							{
//								set_bugao(&leg_d,0,-10,10);
//								set_bugao(&leg_d,2,-10,10);
//								set_bugao(&leg_d,1,-10,10);
//								set_bugao(&leg_d,3,-10,10);
//							}
//							leg_d.state=tort_houtui;
//							motor_danamic_deng();
//						}
//					}
//				}
//			}
//			if(rc_control.s1==3&&flag_zero==1&&rc_control.s2==3)
//			{
//				motor_zhanli();  //站立
//				flag_zero=2;
//			}
//			if(rc_control.s1==2&&flag_zero==0&&rc_control.s2==3)//
//			{
//				motor_encoder();    //发送零力拒进行较零
//			}
			
			
			if(flag_zero_2==0)
			{
			
				motor_set(&motor1[0],0,0,0,0,0);// nm, rads, rad, kp, ks
				motor_send(0,&motor1[0]);
				//flag_zero_2=2;(
			}
			else if(flag_zero_2==1)
			{
//				if((motor_send_go8[0].sen_data.rad-motor_receive_go8[0].rec_data.rad<=500)||(motor_send_go8[0].sen_data.rad-motor_receive_go8[0].rec_data.rad>=-500))
//				{
//					printf("over");
					//nm1=100;
				//}
//				if((motor_send_go8[0].sen_data.rad-motor_receive_go8[0].rec_data.rad>100)||(motor_send_go8[0].sen_data.rad-motor_receive_go8[0].rec_data.rad<-100))
//				{
//					printf("noover");
//					nm1=100;
//				}
				motor_set(&motor1[0],0,100,45*588,100,100);// nm, rads, rad, kp, ks
				motor_send(0,&motor1[0]);
				//flag_zero_2=2;
			}
//			motor_set(&motor1[1],nm2,0,0,0,0);
//			motor_send(1,&motor1[1]);
//			motor_set(&motor1[2],nm3,0,0,0,0);// nm, rads, rad, kp, ks
//			motor_send(2,&motor1[2]);
//			motor_set(&motor1[3],nm4,0,0,0,0);// nm, rads, rad, kp, ks
//			motor_send(3,&motor1[3]);
//			motor_set(&motor1[4],nm1,0,0,0,0);// nm, rads, rad, kp, ks
//			motor_send(4,&motor1[4]);
//			motor_set(&motor1[5],nm2,0,0,0,0);// nm, rads, rad, kp, ks
//			motor_send(5,&motor1[5]);
//			motor_set(&motor1[6],nm3,0,0,0,0);// nm, rads, rad, kp, ks
//			motor_send(6,&motor1[6]);
//			motor_set(&motor1[7],nm4,0,0,0,0);
//			motor_send(7,&motor1[7]);
			rt_thread_mdelay(1);
    }
}

float r,p,y;

void thread_entry2(void* parameter)//2.2
{
	 while(1)
	{  
		//printf("entry2\r\n");
		 //Get_angle(&r,&p,&y);
		//printf("%f %f %f\r\n",r,p,y);
		//printf("%d %d %d %d %d %d \r\n",rc_control.s1,rc_control.s2,rc_control.ch0,rc_control.ch1,rc_control.ch2,rc_control.ch3);
		 //printf(" %f  %d  %f  %d  %d  %d\r\n",nm_jie(0),motor_receive_go8[0].rec_data.rads,nm_jie(1),motor_receive_go8[1].rec_data.rads,a,b);
		//printf("%d %d %d %d\r\n",motor_send_go8[0].sen_data.nm,motor_send_go8[1].sen_data.nm,nm1,nm2);
		//printf("%d %d %d %d %d %d \r\n",motor_receive_go8[0].rec_data.rad,motor_receive_go8[0].rec_data.rads,a_init[0],rc_control.s1,rc_control.s2,rc_control.ch0);//,flag_zero );
		//printf("%d \r\n",motor_receive_go8[0].rec_data.rad);
		//printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n",a_init[0],a_init[1],a_init[6],a_init[7],a_init[2],a_init[3],a_init[4],a_init[5],rc_control.s1 ,flag_zero,motor_receive_go8[3].rec_data.rad,motor_receive_go8[5].rec_data.rad);
		//printf("%d %d\t",motor_receive_go8[4].rec_data.rad,motor_receive_go8[5].rec_data.rad);
		//printf("%f %f %f %f\r\n",leg_d.leg_dex[0],leg_d.leg_dey[0],leg_d.leg_dex[1],leg_d.leg_dey[1]);
		//printf(" %f %f\r\n",mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588+start_zheng+start_ping1,(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588-start_zheng+start_ping2);
		rt_thread_mdelay(10);
	}
}	

void thread_entry3(void* parameter)
{
	 while(1)
	{  
		//printf("entry2\r\n");
		 //Get_angle(&r,&p,&y);
		//printf("%f %f %f\r\n",r,p,y);
		//printf("%d %d %d %d %d %d \r\n",rc_control.s1,rc_control.s2,rc_control.ch0,rc_control.ch1,rc_control.ch2,rc_control.ch3);
		 //printf(" %f  %d  %f  %d  %d  %d\r\n",nm_jie(0),motor_receive_go8[0].rec_data.rads,nm_jie(1),motor_receive_go8[1].rec_data.rads,a,b);
		//printf("%d %d %d %d\r\n",motor_send_go8[0].sen_data.nm,motor_send_go8[1].sen_data.nm,nm1,nm2);
		//printf("%d %d %d %d %d %d \r\n",motor_receive_go8[0].rec_data.rad,motor_receive_go8[0].rec_data.rads,a_init[0],rc_control.s1,rc_control.s2,rc_control.ch0);//,flag_zero );
		//printf("%d \r\n",motor_receive_go8[0].rec_data.rad);
		//printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n",a_init[0],a_init[1],a_init[6],a_init[7],a_init[2],a_init[3],a_init[4],a_init[5],rc_control.s1 ,flag_zero,motor_receive_go8[1].rec_data.rad,motor_receive_go8[7].rec_data.rad);
		//printf("  %d  %f\r\n",flag_zouyou,rhy1_1.S2);
		//printf("%d %d %d %d\r\n",nm1,nm2,nm3,nm4);
		printf("%d %d %d %d\r\n",motor_receive_go8[0].rec_data.rad,motor_send_go8[0].sen_data.rad,nm1,(motor_send_go8[0].sen_data.rad-motor_receive_go8[0].rec_data.rad));
		//printf(" %f %f\r\n",mathematicalmodelparallel_contrary_corner1(dynamic_x,dynamic_y,leg_da,leg_xi,0)*588+start_zheng+start_ping1,(180-mathematicalmodelparallel_contrary_corner2(dynamic_x,dynamic_y,leg_da,leg_xi,0))*588-start_zheng+start_ping2);
		rt_thread_mdelay(10);
	}
}


///*舵轮编码器环*/
void timeout1(void* parameter)
{
//	rhythm(&rhy1,rhy1_1.phase2,rhy1_1.duty2,rhy1_1.H2,rhy1_1.S2,rhy1_1.T2);
//	rhythm(&rhy2,rhy2_2.phase2,rhy2_2.duty2,rhy2_2.H2,rhy2_2.S2,rhy2_2.T2);
//	gait(t1_second,state_motor,state_motor_cong,&rhy1,&rhy2);
	
	
//	//deng_butai_init(&leg_d,0.5,10,15,0.0,27.94,-10,10);
	gait_deng(leg_d.t1_second,&leg_d);
	
	if(rc_control.s1==1&&rc_control.ch1!=0)
	{
		leg_d.t1_second++;
	}
	else
	{
		leg_d.t1_second=0;
	}
	
}



