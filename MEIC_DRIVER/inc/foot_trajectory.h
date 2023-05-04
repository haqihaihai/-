#ifndef __foot_trajectory_H
#define __foot_trajectory_H

typedef struct _RHYTHM{
	float phase2;                 
	float duty2;                 
	float H2;                 
	float S2;						
	float T2;					
}_rhy;


typedef struct _ANGLE{
	float anglenow[8];							
}_angle;

typedef struct zuobiao{
	float leg_dex[4];
	float leg_dey[4];
	float Ts;
	float H;
	float x_qishi[4];
	float x_zhongdian[4];
	float faai;
	float sigma;
	float start_locationx[4];
	float start_locationy[4];
	float guocheng_locationx1[4];
	float guocheng_locationx2[4];
	float guocheng_locationy[4];
	float state;
	float t1_second;
	
	float angle_now_deng[8];
	float last_angle_now_deng[8];
}_legde;



//typedef struct _buchangjia{
//	float a;
//	
//	
//}_buchang;

void rhy_init(_rhy *rhystart);
void rhythm(_rhy *rhystart,float phase,float duty,float H,float S,float T);
void foot_trajectory_complex(int n,float H,float S,float T,float Tmx,float Tmy,float t);
void foot_trajectory_complex8(int n,float H,float S,float T,float Tmx,float Tmy,float t);
void foot_trajectory_complex5(int n,float H,float S,float T,float Tmx,float Tmy,float t);
void gait(float t1,float state,float state_cong,_rhy *rhy_set,_rhy *rhy_set1);

void deng_init(_legde *legdel);
void deng_butai_init(_legde *legdel,float xiangwei,float T,float H,float start_x,float start_y,float qishi_x,float zuizhong_x);
void set_bugao(_legde *legdel_d,int n,float qishi_x,float zuizhong_x);
void gait_deng(float t,_legde *legdel);
void deng_butai_guiji(float t,_legde *legdel);

//void gait_next(float t1);
//void gait_f(float t1);

#endif
