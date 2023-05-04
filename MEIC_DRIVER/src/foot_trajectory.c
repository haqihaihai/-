#include "foot_trajectory.h"
#include "math.h"
#include "mathematicalmadel.h"
#include "usart_go.h"
//#include "force_control.h"

_angle anglenow;
_rhy rhy1;
_rhy rhy2;
_legde leg_de;
float leg_x[4],leg_y[4];


void rhy_init(_rhy *rhystart)
{
	rhystart->phase2=0;//相位差 
	rhystart->duty2=0;	//占空比
	rhystart->H2=0;
	rhystart->S2=0;
	rhystart->T2=0;
}

void rhythm(_rhy *rhystart,float phase,float duty,float H,float S,float T)
{
	rhystart->phase2=phase;//相位差 支撑相系数
	rhystart->duty2=duty;	//占空比
	rhystart->H2=H;
	rhystart->S2=S;
	rhystart->T2=T;
}

void foot_trajectory_complex(int n,float H,float S,float T,float Tmx,float Tmy,float t)
{
	if(t>T)
	{
		t=t-T;
	}
	if(t>=0&&t<Tmx)
	{
		leg_x[n]=S*(t/Tmx-1/(2*PI)*sin(2*PI*t/Tmx));
		if(t>=0&&t<=Tmx/2)
		{
			leg_y[n]=H*(1*(2*(t/Tmx-1/(4*PI)*sin(4*PI*t/Tmx))-1)+1);
		}
		if(t>=Tmx/2&&t<=Tmx)
		{
			leg_y[n]=H*((-1)*(2*(t/Tmx-1/(4*PI)*sin(4*PI*t/Tmx))-1)+1);
		}
	}
	if(t>=Tmx&&t<=T)
	{
		leg_x[n]=S*((T-t)/Tmy+1/(2*PI)*sin(2*PI*t/Tmy));
		if(t>Tmx&&t<=Tmx+Tmy/2)
		{
			leg_y[n]=0;//((-1)*H*(1*(2*((t-Tmx)/Tmy-1/(4*PI)*sin(4*PI*t/Tmy))-1)+1));
		}
		if(t>=Tmx+Tmy/2&&t<=T)
		{
			leg_y[n]=0;//((-1)*H*((-1)*(2*((t-Tmx)/Tmy-1/(4*PI)*sin(4*PI*t/Tmy))-1)+1));
		}
	}
}



//void foot_trajectory_complex8(int n,float H,float S,float T,float Tmx,float Tmy,float t)
//{
//	if(t>=0&&t<Tmx)
//	{
//		leg_x[n]=6*S/pow(Tmx,5)*pow(t,5)-15*S/pow(Tmx,4)*pow(t,4)+10*S/pow(Tmx,3)*pow(t,3);
//		leg_y[n]=(-1)*768*H/pow(Tmx,8)*pow(t,8)+3072*H/pow(Tmx,7)*pow(t,7)-4868*H/pow(Tmx,6)*pow(t,6)+3840*H/pow(Tmx,5)*pow(t,5)-1536*H/pow(Tmx,4)*pow(t,4)+256*H/pow(Tmx,3)*pow(t,3);
//	}
//	if(t>=Tmx&&t<=T)
//	{
//		leg_x[n]=6*S/pow(Tmy,5)*pow((T-t),5)-15*S/pow(Tmy,4)*pow((T-t),4)+10*S/pow(Tmy,3)*pow((T-t),3);
//		leg_y[n]=0;
//	}
//	
//}

//void foot_trajectory_complex5(int n,float H,float S,float T,float Tmx,float Tmy,float t)
//{
//	if(t>=0&&t<Tmx)
//	{
//		leg_x[n]=6*S/pow(Tmx,5)*pow(t,5)-15*S/pow(Tmx,4)*pow(t,4)+10*S/pow(Tmx,3)*pow(t,3);
//		if(t>=0&&t<=Tmx/2)
//		{
//			leg_y[n]=192*H/pow(Tmx,5)*pow(t,5)-240*H/pow(Tmx,4)*pow(t,4)+80*H/pow(Tmx,3)*pow(t,3);
//		}
//		if(t>=Tmx/2&&t<=Tmx)
//		{
//			leg_y[n]=192*H/pow((Tmx-t),5)*pow(t,5)-240*H/pow((Tmx-t),4)*pow(t,4)+80*H/pow((Tmx-t),3)*pow(t,3);
//		}
//	}
//	if(t>=Tmx&&t<=T)
//	{
//		leg_x[n]=6*S/pow(Tmy,5)*pow((T-t),5)-15*S/pow(Tmy,4)*pow((T-t),4)+10*S/pow(Tmy,3)*pow((T-t),3);
//		leg_y[n]=0;
//	}
//}

//int ns[4];
//extern int t1_second;
//int flag_zizhuan1=0;
//int flag_zizhuan2=0;

//void gait(float t1,float state,float state_cong,_rhy *rhy_set,_rhy *rhy_set1)
//{
//	if(state==1)
//	{
//		foot_trajectory_complex(0,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1);
//		ns[0]=0.05+leg_x[0]-rhy_set->S2/2;
//		anglenow.anglenow[0]= mathematicalmodelparallel_contrary_corner1(ns[0],27.94-leg_y[0],13.39,26,0);
//		anglenow.anglenow[1]= mathematicalmodelparallel_contrary_corner2(ns[0],27.94-leg_y[0],13.39,26,0);
//		rad_zhengfu(0,1);
//		
//		foot_trajectory_complex(1,rhy_set1->H2,rhy_set1->S2,rhy_set1->T2,(1-rhy_set1->duty2)*rhy_set1->T2,rhy_set1->duty2*rhy_set1->T2,t1+0.5*rhy_set1->T2);
//		ns[1]=0.05+leg_x[1]-rhy_set1->S2/2;
//		anglenow.anglenow[2]= mathematicalmodelparallel_contrary_corner1(ns[1],27.94-leg_y[1],13.39,26,0);
//		anglenow.anglenow[3]= mathematicalmodelparallel_contrary_corner2(ns[1],27.94-leg_y[1],13.39,26,0);
//		rad_zhengfu(2,3);
//		
//		foot_trajectory_complex(2,rhy_set1->H2,rhy_set1->S2,rhy_set1->T2,(1-rhy_set1->duty2)*rhy_set1->T2,rhy_set1->duty2*rhy_set1->T2,t1);//+rhy_set->phase2*rhy_set->T2
//		ns[2]=0.05+leg_x[2]-rhy_set1->S2/2;
//		anglenow.anglenow[4]= mathematicalmodelparallel_contrary_corner1(ns[2],27.94-leg_y[2],13.39,26,0);
//		anglenow.anglenow[5]= mathematicalmodelparallel_contrary_corner2(ns[2],27.94-leg_y[2],13.39,26,0);
//		rad_zhengfu(4,5);
//		
//		foot_trajectory_complex(3,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+0.5*rhy_set->T2);//t1+(0.5+rhy_set->phase2)
//		ns[3]=0.05+leg_x[3]-rhy_set->S2/2;
//		anglenow.anglenow[6]= mathematicalmodelparallel_contrary_corner1(ns[3],27.94-leg_y[3],13.39,26,0);
//		anglenow.anglenow[7]= mathematicalmodelparallel_contrary_corner2(ns[3],27.94-leg_y[3],13.39,26,0);
//		rad_zhengfu(6,7);
//		
//		if(t1_second>=rhy_set->T2)
//		{
//			t1_second=0;
//		}
//	}
//	else if(state==2)
//	{
//		foot_trajectory_complex(0,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1);
//		ns[0]=0.05+leg_x[0]-rhy_set->S2/2;
//		anglenow.anglenow[0]= mathematicalmodelparallel_contrary_corner1(-ns[0],27.94-leg_y[0],13.39,26,0);
//		anglenow.anglenow[1]= mathematicalmodelparallel_contrary_corner2(-ns[0],27.94-leg_y[0],13.39,26,0);
//		rad_zhengfu(0,1);
//		
//		foot_trajectory_complex(1,rhy_set1->H2,rhy_set1->S2,rhy_set1->T2,(1-rhy_set1->duty2)*rhy_set1->T2,rhy_set1->duty2*rhy_set1->T2,t1+0.5*rhy_set1->T2);
//		ns[1]=0.05+leg_x[1]-rhy_set1->S2/2;
//		anglenow.anglenow[2]= mathematicalmodelparallel_contrary_corner1(-ns[1],27.94-leg_y[1],13.39,26,0);
//		anglenow.anglenow[3]= mathematicalmodelparallel_contrary_corner2(-ns[1],27.94-leg_y[1],13.39,26,0);
//		rad_zhengfu(2,3);
//		
//		foot_trajectory_complex(2,rhy_set1->H2,rhy_set1->S2,rhy_set1->T2,(1-rhy_set1->duty2)*rhy_set1->T2,rhy_set1->duty2*rhy_set1->T2,t1);//+rhy_set->phase2*rhy_set->T2
//		ns[2]=0.05+leg_x[2]-rhy_set1->S2/2;
//		anglenow.anglenow[4]= mathematicalmodelparallel_contrary_corner1(-ns[2],27.94-leg_y[2],13.39,26,0);
//		anglenow.anglenow[5]= mathematicalmodelparallel_contrary_corner2(-ns[2],27.94-leg_y[2],13.39,26,0);
//		rad_zhengfu(4,5);
//		
//		foot_trajectory_complex(3,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+0.5*rhy_set->T2);//t1+(0.5+rhy_set->phase2)
//		ns[3]=0.05+leg_x[3]-rhy_set->S2/2;
//		anglenow.anglenow[6]= mathematicalmodelparallel_contrary_corner1(-ns[3],27.94-leg_y[3],13.39,26,0);
//		anglenow.anglenow[7]= mathematicalmodelparallel_contrary_corner2(-ns[3],27.94-leg_y[3],13.39,26,0);
//		rad_zhengfu(6,7);
//		
//		if(t1_second>=rhy_set->T2)
//		{
//			t1_second=0;
//		}
//	}
//	else if(state==3)
//	{
//		//顺时针
//		if(state_cong==4)
//		{
//			foot_trajectory_complex(0,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1);
//			ns[0]=0.05+leg_x[0]-rhy_set->S2/2;
//			anglenow.anglenow[0]= mathematicalmodelparallel_contrary_corner1(ns[0],27.94-leg_y[0],13.39,26,0);
//			anglenow.anglenow[1]= mathematicalmodelparallel_contrary_corner2(ns[0],27.94-leg_y[0],13.39,26,0);
//			rad_zhengfu(0,1);
//			
//			foot_trajectory_complex(1,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+0.5*rhy_set->T2);
//			ns[1]=0.05+leg_x[1]-rhy_set->S2/2;
//			anglenow.anglenow[2]= mathematicalmodelparallel_contrary_corner1(-ns[1],27.94-leg_y[1],13.39,26,0);
//			anglenow.anglenow[3]= mathematicalmodelparallel_contrary_corner2(-ns[1],27.94-leg_y[1],13.39,26,0);
//			rad_zhengfu(2,3);
//			
//			foot_trajectory_complex(2,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1);//+rhy_set->phase2*rhy_set->T2
//			ns[2]=0.05+leg_x[2]-rhy_set->S2/2;
//			anglenow.anglenow[4]= mathematicalmodelparallel_contrary_corner1(ns[2],27.94-leg_y[2],13.39,26,0);
//			anglenow.anglenow[5]= mathematicalmodelparallel_contrary_corner2(ns[2],27.94-leg_y[2],13.39,26,0);
//			rad_zhengfu(4,5);
//			
//			foot_trajectory_complex(3,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+0.5*rhy_set->T2);//t1+(0.5+rhy_set->phase2)
//			ns[3]=0.05+leg_x[3]-rhy_set->S2/2;
//			anglenow.anglenow[6]= mathematicalmodelparallel_contrary_corner1(-ns[3],27.94-leg_y[3],13.39,26,0);
//			anglenow.anglenow[7]= mathematicalmodelparallel_contrary_corner2(-ns[3],27.94-leg_y[3],13.39,26,0);
//			rad_zhengfu(6,7);
//		}	
//		else if(state_cong==5)
//		{
//			foot_trajectory_complex(0,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+0.5*rhy_set->T2);
//			ns[0]=0.05+leg_x[0]-rhy_set->S2/2;
//			anglenow.anglenow[0]= mathematicalmodelparallel_contrary_corner1(ns[0],27.94-leg_y[0],13.39,26,0);
//			anglenow.anglenow[1]= mathematicalmodelparallel_contrary_corner2(ns[0],27.94-leg_y[0],13.39,26,0);
//			rad_zhengfu(0,1);
//			
//			foot_trajectory_complex(1,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1);
//			ns[1]=0.05+leg_x[1]-rhy_set->S2/2;
//			anglenow.anglenow[2]= mathematicalmodelparallel_contrary_corner1(-ns[1],27.94-leg_y[1],13.39,26,0);
//			anglenow.anglenow[3]= mathematicalmodelparallel_contrary_corner2(-ns[1],27.94-leg_y[1],13.39,26,0);
//			rad_zhengfu(2,3);
//			
//			foot_trajectory_complex(2,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+0.5*rhy_set->T2);//+rhy_set->phase2*rhy_set->T2
//			ns[2]=0.05+leg_x[2]-rhy_set->S2/2;
//			anglenow.anglenow[4]= mathematicalmodelparallel_contrary_corner1(ns[2],27.94-leg_y[2],13.39,26,0);
//			anglenow.anglenow[5]= mathematicalmodelparallel_contrary_corner2(ns[2],27.94-leg_y[2],13.39,26,0);
//			rad_zhengfu(4,5);
//			
//			foot_trajectory_complex(3,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1);//t1+(0.5+rhy_set->phase2)
//			ns[3]=0.05+leg_x[3]-rhy_set->S2/2;
//			anglenow.anglenow[6]= mathematicalmodelparallel_contrary_corner1(-ns[3],27.94-leg_y[3],13.39,26,0);
//			anglenow.anglenow[7]= mathematicalmodelparallel_contrary_corner2(-ns[3],27.94-leg_y[3],13.39,26,0);
//			rad_zhengfu(6,7);
//		}
//		
//		if(t1_second>=rhy_set->T2)
//		{
//			t1_second=0;
//		}
//	}
//}


//灯哥 步态+轨迹

void deng_init(_legde *legdel)
{
	legdel->angle_now_deng[0]=0;
	legdel->angle_now_deng[1]=0;
	legdel->angle_now_deng[2]=0;
	legdel->angle_now_deng[4]=0;
	legdel->angle_now_deng[5]=0;
	legdel->angle_now_deng[6]=0;
	legdel->angle_now_deng[7]=0;
	legdel->faai=0;
	legdel->guocheng_locationx1[0]=0;
	legdel->guocheng_locationx2[0]=0;
	legdel->guocheng_locationy[0]=0;
	legdel->guocheng_locationx1[1]=0;
	legdel->guocheng_locationx2[1]=0;
	legdel->guocheng_locationy[1]=0;
	legdel->guocheng_locationx1[2]=0;
	legdel->guocheng_locationx2[2]=0;
	legdel->guocheng_locationy[2]=0;
	legdel->guocheng_locationx1[3]=0;
	legdel->guocheng_locationx2[3]=0;
	legdel->guocheng_locationy[3]=0;
	legdel->H=0;
	legdel->last_angle_now_deng[0]=0;
	legdel->last_angle_now_deng[1]=0;
	legdel->last_angle_now_deng[2]=0;
	legdel->last_angle_now_deng[3]=0;
	legdel->last_angle_now_deng[4]=0;
	legdel->last_angle_now_deng[5]=0;
	legdel->last_angle_now_deng[6]=0;
	legdel->last_angle_now_deng[7]=0;
	legdel->leg_dex[0]=0;
	legdel->leg_dex[1]=0;
	legdel->leg_dex[2]=0;
	legdel->leg_dex[3]=0;
	legdel->leg_dey[0]=0;
	legdel->leg_dey[1]=0;
	legdel->leg_dey[2]=0;
	legdel->leg_dey[3]=0;
	legdel->sigma=0;
	legdel->start_locationx[0]=0;
	legdel->start_locationy[0]=0;
	legdel->start_locationx[1]=0;
	legdel->start_locationy[1]=0;
	legdel->start_locationx[2]=0;
	legdel->start_locationy[2]=0;
	legdel->start_locationx[3]=0;
	legdel->start_locationy[3]=0;
	legdel->t1_second=0;
	legdel->state=0;
	legdel->Ts=0;
	legdel->x_qishi[0]=0;
	legdel->x_zhongdian[0]=0;
	legdel->x_qishi[1]=0;
	legdel->x_zhongdian[1]=0;
	legdel->x_qishi[2]=0;
	legdel->x_zhongdian[2]=0;
	legdel->x_qishi[3]=0;
	legdel->x_zhongdian[3]=0;
}

void deng_butai_init(_legde *legdel,float xiangwei,float T,float H,float start_x,float start_y,float qishi_x,float zuizhong_x)
{
	legdel->faai=xiangwei ;
	legdel->Ts=T;
	legdel->H=H;
	legdel->start_locationy[0]=start_y;
	legdel->start_locationy[1]=start_y;
	legdel->start_locationy[2]=start_y;
	legdel->start_locationy[3]=start_y;
	legdel->leg_dex[0]=start_x;
	legdel->leg_dex[1]=start_x;
	legdel->leg_dex[2]=start_x;
	legdel->leg_dex[3]=start_x;
	legdel->x_qishi[0]=qishi_x;
	legdel->x_zhongdian[0]=zuizhong_x ;
	legdel->x_qishi[1]=qishi_x;
	legdel->x_zhongdian[1]=zuizhong_x ;
	legdel->x_qishi[2]=qishi_x;
	legdel->x_zhongdian[2]=zuizhong_x ;
	legdel->x_qishi[3]=qishi_x;
	legdel->x_zhongdian[3]=zuizhong_x ;
}

void set_bugao(_legde *legdel_d,int n,float qishi_x,float zuizhong_x)
{
	legdel_d->x_qishi[n]=qishi_x;
	legdel_d->x_zhongdian[n]=zuizhong_x;
}

void gait_deng(float t,_legde *legdel_d)
{
	deng_butai_guiji(t,legdel_d);
	if(legdel_d->state==1)
	{
		legdel_d->angle_now_deng[0]=mathematicalmodelparallel_contrary_corner1(legdel_d->leg_dex[0],legdel_d->leg_dey[0],13.39,26,0);
		legdel_d->angle_now_deng[1]=mathematicalmodelparallel_contrary_corner2(legdel_d->leg_dex[0],legdel_d->leg_dey[0],13.39,26,0);
		rad_zhengfu_deng(0,1,legdel_d);
		
		legdel_d->angle_now_deng[2]=mathematicalmodelparallel_contrary_corner1(legdel_d->leg_dex[1],legdel_d->leg_dey[1],13.39,26,0);
		legdel_d->angle_now_deng[3]=mathematicalmodelparallel_contrary_corner2(legdel_d->leg_dex[1],legdel_d->leg_dey[1],13.39,26,0);
		rad_zhengfu_deng(2,3,legdel_d);
		
		legdel_d->angle_now_deng[4]=mathematicalmodelparallel_contrary_corner1(legdel_d->leg_dex[2],legdel_d->leg_dey[2],13.39,26,0);
		legdel_d->angle_now_deng[5]=mathematicalmodelparallel_contrary_corner2(legdel_d->leg_dex[2],legdel_d->leg_dey[2],13.39,26,0);
		rad_zhengfu_deng(4,5,legdel_d);
		
		legdel_d->angle_now_deng[6]=mathematicalmodelparallel_contrary_corner1(legdel_d->leg_dex[3],legdel_d->leg_dey[3],13.39,26,0);
		legdel_d->angle_now_deng[7]=mathematicalmodelparallel_contrary_corner2(legdel_d->leg_dex[3],legdel_d->leg_dey[3],13.39,26,0);
		rad_zhengfu_deng(6,7,legdel_d);
	
//		legdel_d->angle_now_deng[0]=mathematicalmodelparallel_contrary_corner1(0,legdel_d->leg_dey[0],13.39,26,0);
//		legdel_d->angle_now_deng[1]=mathematicalmodelparallel_contrary_corner2(0,legdel_d->leg_dey[0],13.39,26,0);
//		rad_zhengfu_deng(0,1,legdel_d);
//		
//		legdel_d->angle_now_deng[2]=mathematicalmodelparallel_contrary_corner1(0,legdel_d->leg_dey[1],13.39,26,0);
//		legdel_d->angle_now_deng[3]=mathematicalmodelparallel_contrary_corner2(0,legdel_d->leg_dey[1],13.39,26,0);
//		rad_zhengfu_deng(2,3,legdel_d);
//		
//		legdel_d->angle_now_deng[4]=mathematicalmodelparallel_contrary_corner1(0,legdel_d->leg_dey[2],13.39,26,0);
//		legdel_d->angle_now_deng[5]=mathematicalmodelparallel_contrary_corner2(0,legdel_d->leg_dey[2],13.39,26,0);
//		rad_zhengfu_deng(4,5,legdel_d);
//		
//		legdel_d->angle_now_deng[6]=mathematicalmodelparallel_contrary_corner1(0,legdel_d->leg_dey[3],13.39,26,0);
//		legdel_d->angle_now_deng[7]=mathematicalmodelparallel_contrary_corner2(0,legdel_d->leg_dey[3],13.39,26,0);
//		rad_zhengfu_deng(6,7,legdel_d);
		
		if(legdel_d->t1_second>=legdel_d->Ts)
		{
			legdel_d->t1_second=0;
		}
	}
	else if(legdel_d->state==2)
	{
		legdel_d->angle_now_deng[0]=mathematicalmodelparallel_contrary_corner1(-legdel_d->leg_dex[0],legdel_d->leg_dey[0],13.39,26,0);
		legdel_d->angle_now_deng[1]=mathematicalmodelparallel_contrary_corner2(-legdel_d->leg_dex[0],legdel_d->leg_dey[0],13.39,26,0);
		rad_zhengfu_deng(0,1,legdel_d);
		
		legdel_d->angle_now_deng[2]=mathematicalmodelparallel_contrary_corner1(-legdel_d->leg_dex[1],legdel_d->leg_dey[1],13.39,26,0);
		legdel_d->angle_now_deng[3]=mathematicalmodelparallel_contrary_corner2(-legdel_d->leg_dex[1],legdel_d->leg_dey[1],13.39,26,0);
		rad_zhengfu_deng(2,3,legdel_d);
		
		legdel_d->angle_now_deng[4]=mathematicalmodelparallel_contrary_corner1(-legdel_d->leg_dex[2],legdel_d->leg_dey[2],13.39,26,0);
		legdel_d->angle_now_deng[5]=mathematicalmodelparallel_contrary_corner2(-legdel_d->leg_dex[2],legdel_d->leg_dey[2],13.39,26,0);
		rad_zhengfu_deng(4,5,legdel_d);
		
		legdel_d->angle_now_deng[6]=mathematicalmodelparallel_contrary_corner1(-legdel_d->leg_dex[3],legdel_d->leg_dey[3],13.39,26,0);
		legdel_d->angle_now_deng[7]=mathematicalmodelparallel_contrary_corner2(-legdel_d->leg_dex[3],legdel_d->leg_dey[3],13.39,26,0);
		rad_zhengfu_deng(6,7,legdel_d);
		
		if(legdel_d->t1_second>=legdel_d->Ts)
		{
			legdel_d->t1_second=0;
		}
	}
	
	
}

void deng_butai_guiji(float t,_legde *legdel)
{
	if(t<=legdel->Ts*legdel ->faai)
	{
		legdel->sigma=2*PI*t/(legdel->faai*legdel->Ts);
		legdel->guocheng_locationy[0]=legdel->H*(1-cos(legdel->sigma))/2;
		legdel->guocheng_locationy[1]=legdel->H*(1-cos(legdel->sigma))/2;
		legdel->guocheng_locationy[2]=legdel->H*(1-cos(legdel->sigma))/2;
		legdel->guocheng_locationy[3]=legdel->H*(1-cos(legdel->sigma))/2;
		legdel->guocheng_locationx1[0]=(legdel->x_zhongdian[0]-legdel->x_qishi[0])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_qishi[0];
		legdel->guocheng_locationx2[0]=(legdel->x_qishi[0]-legdel->x_zhongdian[0])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_zhongdian[0];
		legdel->guocheng_locationx1[1]=(legdel->x_zhongdian[1]-legdel->x_qishi[1])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_qishi[1];
		legdel->guocheng_locationx2[1]=(legdel->x_qishi[1]-legdel->x_zhongdian[1])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_zhongdian[1];
		legdel->guocheng_locationx1[2]=(legdel->x_zhongdian[2]-legdel->x_qishi[2])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_qishi[2];
		legdel->guocheng_locationx2[2]=(legdel->x_qishi[2]-legdel->x_zhongdian[2])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_zhongdian[2];
		legdel->guocheng_locationx1[3]=(legdel->x_zhongdian[3]-legdel->x_qishi[3])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_qishi[3];
		legdel->guocheng_locationx2[3]=(legdel->x_qishi[3]-legdel->x_zhongdian[3])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_zhongdian[3];
//		
//		//输出y
//		legdel->leg_dey[0]=legdel->start_locationy[0]-legdel->guocheng_locationy;
//		legdel->leg_dey[1]=legdel->start_locationy[1];
//		legdel->leg_dey[2]=legdel->start_locationy[2]-legdel->guocheng_locationy;
//		legdel->leg_dey[3]=legdel->start_locationy[3];
//		
//		//输出x
//		legdel->leg_dex[0]=-legdel->guocheng_locationx2;
//		legdel->leg_dex[1]=-legdel->guocheng_locationx1;
//		legdel->leg_dex[2]=-legdel->guocheng_locationx2;
//		legdel->leg_dex[3]=-legdel->guocheng_locationx1;
		
		
//		//改的迈腿顺序
//		//输出y
//		legdel->leg_dey[0]=legdel->start_locationy[0];
//		legdel->leg_dey[1]=legdel->start_locationy[1]-legdel->guocheng_locationy;
//		legdel->leg_dey[2]=legdel->start_locationy[2];
//		legdel->leg_dey[3]=legdel->start_locationy[3]-legdel->guocheng_locationy;
//		
//		//输出x
//		legdel->leg_dex[0]=-legdel->guocheng_locationx1;
//		legdel->leg_dex[1]=-legdel->guocheng_locationx2;
//		legdel->leg_dex[2]=-legdel->guocheng_locationx1;
//		legdel->leg_dex[3]=-legdel->guocheng_locationx2;

		legdel->leg_dey[0]=legdel->start_locationy[0];
		legdel->leg_dey[1]=legdel->start_locationy[1]-legdel->guocheng_locationy[1];
		legdel->leg_dey[2]=legdel->start_locationy[2]-legdel->guocheng_locationy[2];
		legdel->leg_dey[3]=legdel->start_locationy[3];
		
		//输出x
		legdel->leg_dex[0]=-legdel->guocheng_locationx1[0];
		legdel->leg_dex[1]=-legdel->guocheng_locationx2[1];
		legdel->leg_dex[2]=-legdel->guocheng_locationx2[2];
		legdel->leg_dex[3]=-legdel->guocheng_locationx1[3];
		
		
	}
	else if(t>legdel->Ts*legdel ->faai&&t<legdel->Ts)
	{
		legdel->sigma=2*PI*(t-legdel->faai*legdel->Ts)/(legdel->faai*legdel->Ts);
		legdel->guocheng_locationy[0]=legdel->H*(1-cos(legdel->sigma))/2;
		legdel->guocheng_locationy[1]=legdel->H*(1-cos(legdel->sigma))/2;
		legdel->guocheng_locationy[2]=legdel->H*(1-cos(legdel->sigma))/2;
		legdel->guocheng_locationy[3]=legdel->H*(1-cos(legdel->sigma))/2;
		legdel->guocheng_locationx1[0]=(legdel->x_zhongdian[0]-legdel->x_qishi[0])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_qishi[0];
		legdel->guocheng_locationx2[0]=(legdel->x_qishi[0]-legdel->x_zhongdian[0])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_zhongdian[0];
		legdel->guocheng_locationx1[1]=(legdel->x_zhongdian[1]-legdel->x_qishi[1])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_qishi[1];
		legdel->guocheng_locationx2[1]=(legdel->x_qishi[1]-legdel->x_zhongdian[1])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_zhongdian[1];
		legdel->guocheng_locationx1[2]=(legdel->x_zhongdian[2]-legdel->x_qishi[2])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_qishi[2];
		legdel->guocheng_locationx2[2]=(legdel->x_qishi[2]-legdel->x_zhongdian[2])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_zhongdian[2];
		legdel->guocheng_locationx1[3]=(legdel->x_zhongdian[3]-legdel->x_qishi[3])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_qishi[3];
		legdel->guocheng_locationx2[3]=(legdel->x_qishi[3]-legdel->x_zhongdian[3])
									 *((legdel->sigma-sin(legdel->sigma))/(2*PI))+legdel->x_zhongdian[3];
		
//		//输出y
//		legdel->leg_dey[0]=legdel->start_locationy[0];
//		legdel->leg_dey[1]=legdel->start_locationy[1]-legdel->guocheng_locationy;
//		legdel->leg_dey[2]=legdel->start_locationy[2];
//		legdel->leg_dey[3]=legdel->start_locationy[3]-legdel->guocheng_locationy;
//		
//		//输出x
//		legdel->leg_dex[0]=-legdel->guocheng_locationx1;
//		legdel->leg_dex[1]=-legdel->guocheng_locationx2;
//		legdel->leg_dex[2]=-legdel->guocheng_locationx1;
//		legdel->leg_dex[3]=-legdel->guocheng_locationx2;
		
//		
//		//改的迈腿顺序
//		legdel->leg_dey[0]=legdel->start_locationy[0]-legdel->guocheng_locationy;
//		legdel->leg_dey[1]=legdel->start_locationy[1];
//		legdel->leg_dey[2]=legdel->start_locationy[2]-legdel->guocheng_locationy;
//		legdel->leg_dey[3]=legdel->start_locationy[3];
//		
//		//输出x
//		legdel->leg_dex[0]=-legdel->guocheng_locationx2;
//		legdel->leg_dex[1]=-legdel->guocheng_locationx1;
//		legdel->leg_dex[2]=-legdel->guocheng_locationx2;
//		legdel->leg_dex[3]=-legdel->guocheng_locationx1;

		legdel->leg_dey[0]=legdel->start_locationy[0]-legdel->guocheng_locationy[0];
		legdel->leg_dey[1]=legdel->start_locationy[1];
		legdel->leg_dey[2]=legdel->start_locationy[2];
		legdel->leg_dey[3]=legdel->start_locationy[3]-legdel->guocheng_locationy[3];
		
		//输出x
		legdel->leg_dex[0]=-legdel->guocheng_locationx2[0];
		legdel->leg_dex[1]=-legdel->guocheng_locationx1[1];
		legdel->leg_dex[2]=-legdel->guocheng_locationx1[2];
		legdel->leg_dex[3]=-legdel->guocheng_locationx2[3];
	}
}

//void gait_next(float t1)
//{
//	foot_trajectory_complex(0,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+1);
//	anglenow_next.anglenow[0]= mathematicalmodelparallel_contrary_corner1(0.002+leg_x[0],27.713-leg_y[0],9.7,27,4.2);
//	anglenow_next.anglenow[1]= mathematicalmodelparallel_contrary_corner2(0.002+leg_x[0],27.713-leg_y[0],9.7,27,4.2);
//	foot_trajectory_complex(1,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+1+0.5*rhy_set->T2);
//	anglenow_next.anglenow[2]= mathematicalmodelparallel_contrary_corner1(0.002+leg_x[1],27.713-leg_y[1],9.7,27,4.2);
//	anglenow_next.anglenow[3]= mathematicalmodelparallel_contrary_corner2(0.002+leg_x[1],27.713-leg_y[1],9.7,27,4.2);
//	foot_trajectory_complex(2,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+1+rhy_set->phase2*rhy_set->T2);
//	anglenow_next.anglenow[4]= mathematicalmodelparallel_contrary_corner1(0.002+leg_x[2],27.713-leg_y[2],9.7,27,4.2);
//	anglenow_next.anglenow[5]= mathematicalmodelparallel_contrary_corner2(0.002+leg_x[2],27.713-leg_y[2],9.7,27,4.2);
//	foot_trajectory_complex(3,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+1+(0.5+rhy_set->phase2)*rhy_set->T2);
//	anglenow_next.anglenow[6]= mathematicalmodelparallel_contrary_corner1(0.002+leg_x[3],27.713-leg_y[3],9.7,27,4.2);
//	anglenow_next.anglenow[7]= mathematicalmodelparallel_contrary_corner2(0.002+leg_x[3],27.713-leg_y[3],9.7,27,4.2);
//}

//void gait_f(float t1)
//{
//	foot_trajectory_complex(poselec.x1,poselec.y1,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1);
//	foot_trajectory_complex(poselec.x2,poselec.y2,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+0.5*rhy_set->T2);
//	foot_trajectory_complex(poselec.x3,poselec.y3,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+rhy_set->phase2*rhy_set->T2);
//	foot_trajectory_complex(poselec.x4,poselec.y4,rhy_set->H2,rhy_set->S2,rhy_set->T2,(1-rhy_set->duty2)*rhy_set->T2,rhy_set->duty2*rhy_set->T2,t1+(0.5+rhy_set->phase2)*rhy_set->T2);
//}

