#include "ChassisControl.h"
#include "stdio.h"


	float yaw_ppiidd;
	float yaw_ppiiddv;
   int   claw_sign=2;
   float H_L=0,L_R=0;
   float wheelspeed[4]={000.0,000.0,000.0,000.0};
   float wheelposition[4]={0.0,0.0,0.0,0.0};
//   float yaw_ppiidd=0,yaw_ppiiddv=0;
	_pid wheel[4]={ {1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0},\
			  	    {1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0},\
				    {1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0},\
				    {1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0}};
		_pid wheel0[4]={ {1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0},\
			  	    {1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0},\
				    {1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0},\
				    {1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0}};
//	_pid pid_position[4]={ {180,11,400,0,0,0,0,0,8000,4000,0,0},\
//			  	           {180,11,400,0,0,0,0,0,8000,4000,0,0},\
//				           {180,11,400,0,0,0,0,0,8000,4000,0,0},\
//				           {180,11,400,0,0,0,0,0,8000,4000,0,0}};
					
	_pid pid_position[2]={ {3,0,0,0,0,0,0,0,8000,4000,0,0},\
			  	           {3,0,0,0,0,0,0,0,8000,4000,0,0},\
				           };
	
	_pid pid_pos[2]={ {2.5,0.0,0,0,0,0,0,0,8000,4000,0,0},\
			  	          {2.5,0.0,0,0,0,0,0,0,8000,4000,0,0},\
				           };			
	_pid pid_yaw={45,0.0,0.0,0,0,0,0,0,8000,300000,0,0};		
	_pid pid_yawv={1,0.0,0.0,0,0,0,0,0,8000,3000,0,0};	
	
	
	
	_pid pid_claw[2]={ {10,0.0,0.0,0,0,0,0,0,8000,2000,0,0},\
					           {10,0.0,0.0,0,0,0,0,0,8000,2000,0,0}};
	
	_pid pid_claw_position[2]={ {10,0,10,0,0,0,0,0,8000,4000,0,0},\
								{10,0,10,0,0,0,0,0,8000,4000,0,0}};
	
	
//	_pid pid_claw[2]={ {0.89*5/2,0.008*5/2,0.001*5/2,0,0,0,0,0,8000,32000,0,0},\
//					   {0.89*5/2,0.008*5/2,0.001*5/2,0,0,0,0,0,8000,32000,0,0}};
//	
//	_pid pid_claw_position[2]={ {10*2/5,0,10*5/2,0,0,0,0,0,8000,4000*2,0,0},\
//								{10*2/5,0,10*5/2,0,0,0,0,0,8000,4000*2,0,0}};
								
								
//	_pid pid_claw_position[2]={ {0.1,0,0,0,0,0,0,0,8000,4000,0,0},\
//								{0.1,0,0,0,0,0,0,0,8000,4000,0,0}};
								
	_pid pid_LR=			{ 1.9,0.06,0.08,0,0,0,0,0,8000,23000,0,0};
	_pid pid_LR_position=	{ 90,0,40,0,0,0,0,0,8000,3000,0,0};
	
	_pid pid_HL=			{ 1.9,0.06,0.08,0,0,0,0,0,8000,32000,0,0};
	_pid pid_HL_position=	{ 90,0,40,0,0,0,0,0,8000,4000,0,0};
	
	_pid pid_Lidar=			{ 10,0,0.08,0,0,0,0,0,8000,800,0,0};
	
 _pid pid_Lidar_position=  { 15.0,0,90,0,0,0,0,0,8000,2000,0,0};
 _pid pid_Lidar_position_H={ 15.0,0,90,0,0,0,0,0,8000,2000,0,0};
 _pid Pos_pidx={ 9.0,0,10,0,0,0,0,0,8000,2000,0,0};
 _pid Pos_pidy={ 9.0,0,10,0,0,0,0,0,8000,3000,0,0};
 _pid Pos_pidx1={ 13.0,0,100,0,0,0,0,0,8000,2000,0,0};
 
 
	
float posxy1[30][2]={
{(1500.000000-1500)  ,  3650.000000-3650},
{(1500.000000-1500)  ,  3453.571429-3650},
{(1500.000000-1500)  ,  3257.142857-3650},
{(1500.000000-1500)  ,  3060.714286-3650},
{(1500.000000-1500)  ,  2864.285714-3650},
{(1500.000000-1500)  ,  2667.857143-3650},
{(1500.000000-1500)  ,  2471.428571-3650},
{(1500.000000-1500)  ,  2275.000000-3650},
{(1500.000000-1500)  ,  2078.571429-3650},
{(1500.000000-1500)  ,  1882.142857-3650},
{(1500.000000-1500)  ,  1685.714286-3650},
{(1500.000000-1500)  ,  1489.285714-3650},
{(1500.000000-1500)  ,  1292.857143-3650},
{(1500.000000-1500)  ,  1096.428571-3650},
{(1500.000000-1500)  ,  900.000000-3650},
};
	
float posxy2[30][2]={
{(500.000000-500)  ,  900.000000-900},
{(500.000000-500)  ,  1163.023330-900},
{(500.000000-500)  ,  1380.042485-900},
{(525.784361-500)  ,  1555.860601-900},
{(614.609443-500)  ,  1697.417832-900},
{(766.777656-500)  ,  1804.770512-900},
{(982.289001-500)  ,  1877.918639-900},
{(1233.061950-500)  ,  1932.264922-900},
{(1428.337264-500)  ,  2017.590767-900},
{(1561.195417-500)  ,  2137.691533-900},
{(1631.636407-500)  ,  2292.567219-900},
{(1639.660235-500)  ,  2482.217825-900},
{(1607.237137-500)  ,  2705.619828-900},
{(1560.666380-500)  ,  2961.548028-900},
{(1500.000000-500)  ,  3250.000000-900},
};

float posxy3[30][2]={//2-1
{(1500.000000-1500)  ,  900.000000-900},
{(1500.000000-1500)  ,  1156.000192-900},
{(1500.000000-1500)  ,  1372.385607-900},
{(1479.517034-1500)  ,  1550.824111-900},
{(1399.709763-1500)  ,  1694.478435-900},
{(1259.651050-1500)  ,  1803.424075-900},
{(1059.340895-1500)  ,  1877.661029-900},
{(825.180710-1500)  ,  1932.953549-900},
{(641.455455-1500)  ,  2019.628087-900},
{(514.464370-1500)  ,  2141.445908-900},
{(444.207454-1500)  ,  2298.407015-900},
{(430.659192-1500)  ,  2490.508341-900},
{(447.193429-1500)  ,  2714.552242-900},
{(470.307031-1500)  ,  2967.716129-900},
{(500.000000-1500)  ,  3250.000000-900},
};
	
float pos_s[30]={
2750.000000,
2553.571429,
2357.142857,
2160.714286,
1964.285714,
1767.857143,
1571.428571,
1375.000000,
1178.571429,
982.142857,
785.714286,
589.285714,
392.857143,
196.428571,
0.000000,
};
float pos_s2[30]={
3028.063140,
2765.039810,
2548.020655,
2370.321912,
2203.204155,
2016.979122,
1789.392241,
1532.798028,
1319.694887,
1140.598593,
970.456267,
780.635999,
554.893419,
294.762531,
0.000000,


};
	
float pos_s3[30]={//2-1
2954.369589,
2698.369397,
2481.983982,
2302.373705,
2138.039395,
1960.597587,
1746.973422,
1506.373640,
1303.229739,
1127.257248,
955.289742,
762.711251,
538.058073,
283.841252,
0.000000,

};
void Speed_XY(float v_x,float v_y){
	float yaw_pid=0,yaw_ppiidd=0,yaw_ppiiddv=0,raw=0;
	raw=(double)ACTION_GL_POS_DATA.ANGLE_Z/180*PI;
	
	
	
	pid_yaw.SetSpeed=0;
	
//	pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
//	yaw_ppiidd=PID_YAW(&pid_yaw);
//	pid_yawv.SetSpeed=yaw_ppiidd;
//	pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
//	yaw_ppiiddv=PID_YAWV(&pid_yaw);
	
	
	for(int i=0;i<4;i++){
								wheelspeed[i]=0;	
							}
						
							wheelspeed[0]+=(int)((double)v_y*arm_cos_f32(PI*1/4));
							wheelspeed[1]+=(int)((double)v_y*arm_cos_f32(PI*3/4));
							wheelspeed[2]+=(int)((double)v_y*arm_cos_f32(PI*5/4));
							wheelspeed[3]+=(int)((double)v_y*arm_cos_f32(PI*7/4));
						
						
							wheelspeed[0]+=(int)((double)v_x*arm_sin_f32(PI*1/4));
							wheelspeed[1]+=(int)((double)v_x*arm_sin_f32(PI*3/4));
							wheelspeed[2]+=(int)((double)v_x*arm_sin_f32(PI*5/4));
							wheelspeed[3]+=(int)((double)v_x*arm_sin_f32(PI*7/4));
						
			pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
			yaw_ppiidd=PID_YAW(&pid_yaw);
			pid_yawv.SetSpeed=yaw_ppiidd;
			pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
			yaw_ppiiddv=PID_YAWV(&pid_yawv);
			
							
			for(int i=0;i<4;i++){
				wheel[i].SetSpeed=0;

			}				
				wheel[0].SetSpeed=yaw_ppiiddv;
				wheel[1].SetSpeed=yaw_ppiiddv;
				wheel[2].SetSpeed=yaw_ppiiddv;
				wheel[3].SetSpeed=yaw_ppiiddv;
				
			
							
			for(int i=0;i<4;i++){
				wheel[i].SetSpeed+=wheelspeed[i];
				wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
				elecurrent[i]=(int)PID_Calc(&wheel[i]);	//printf("elecurrent[1]=%d",elecurrent[1]);
				
			}
			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);	
	

}
int a_c_num=0;
float a_c_last_speed=0;
int Auto_Control(float x_y[][2],float distance[],int point_num,float speed_max,float x_now,float y_now){
	float acutual_x=0,acutual_y=0,x=0,y=0,r=0,r_square=0,acc_v=48,s=0,s_square=0;
	
	acutual_x=ACTION_GL_POS_DATA.POS_X-x_now;
	acutual_y=ACTION_GL_POS_DATA.POS_Y-y_now;
	
	x=x_y[a_c_num][0]-acutual_x;
	y=x_y[a_c_num][1]-acutual_y;
	r_square=x*x+y*y;
	
	if(a_c_num>=point_num-2){
		a_c_num=0;
		return 0;
	}else{
		while(r_square<400||\
			((x_y[a_c_num+1][0]-x_y[a_c_num][0])*x+
		(x_y[a_c_num+1][1]-x_y[a_c_num][1])*y)<0){
				a_c_num++;
				x=x_y[a_c_num][0]-acutual_x;
				y=x_y[a_c_num][1]-acutual_y;
				r_square=x*x+y*y;
		}
		
//		while(r_square<400||\
//			((x_y[point_num-1][0]-x_y[a_c_num][0])*x+
//		(x_y[point_num-1][1]-x_y[a_c_num][1])*y)<0){
//				a_c_num++;
//				x=x_y[a_c_num][0]-acutual_x;
//				y=x_y[a_c_num][1]-acutual_y;
//				r_square=x*x+y*y;
//		}
		
		
		
		s=distance[a_c_num];
		s_square=s*s;
		r=(float)sqrt((double)r_square);
		
		if(((s_square/1000)+900)<speed_max||\
			((s_square/1000)+900)<a_c_last_speed){
			if(s<100){
				a_c_num=0;
				return 0;
			}
			speed_max=((s_square/1000)+900);
			
			if(speed_max-a_c_last_speed>acc_v){
				speed_max=a_c_last_speed+acc_v;
			}
			a_c_last_speed=speed_max;
			Speed_XY(x/r*speed_max,y/r*speed_max);
			return 1;
			
		}
		if(speed_max-a_c_last_speed>acc_v){
			speed_max=a_c_last_speed+acc_v;
		}
		a_c_last_speed=speed_max;
		Speed_XY(x/r*speed_max,y/r*speed_max);
		//PRINTF("%f,%f,%f\r\n",x,y,r);
		return 1;
		
		
	}
		
}























int i1=0;	
void Pos_control(float x_y[][2],int point_num,float speed){
	int i=0;
	float r=0,x=0,y=0,x_s=0,y_s=0,raw=0,raw2=0,yaw_ppiidd=0,yaw_ppiiddv=0;	
	speed=400;
		raw=(double)ACTION_GL_POS_DATA.ANGLE_Z/180*PI;
		x=ACTION_GL_POS_DATA.POS_X;
		y=ACTION_GL_POS_DATA.POS_Y;
		
		


	if(i1<28){
			r=(x_y[i1][0]-x)*(x_y[i1][0]-x)+(x_y[i1][1]-y)*(x_y[i1][1]-y);//printf("%f\r\n",r);
			x_s=(x_y[i1][0]-x)/sqrt(r)*speed;
			y_s=(x_y[i1][1]-y)/sqrt(r)*speed;
		if(r<900||((x_y[i1+1][0]-x_y[i1][0])*(x_y[i1][0]-x)+(x_y[i1+1][1]-x_y[i1][1])*(x_y[i1][1]-y))<0){
			i1++;
		if(i1>=10&&i1<20){
			pid_yaw.SetSpeed+=18;
		}
		//printf("i1=%d\r\n",i1);
			
	//Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
			
		}
		
	
			
			for(i=0;i<4;i++){
								wheelspeed[i]=0;	
							}
						
							wheelspeed[0]+=(int)((double)y_s*3*arm_cos_f32(PI*1/4+raw));
							wheelspeed[1]+=(int)((double)y_s*3*arm_cos_f32(PI*3/4+raw));
							wheelspeed[2]+=(int)((double)y_s*3*arm_cos_f32(PI*5/4+raw));
							wheelspeed[3]+=(int)((double)y_s*3*arm_cos_f32(PI*7/4+raw));
						
						
							wheelspeed[0]+=(int)((double)x_s*3*arm_sin_f32(PI*1/4+raw));
							wheelspeed[1]+=(int)((double)x_s*3*arm_sin_f32(PI*3/4+raw));
							wheelspeed[2]+=(int)((double)x_s*3*arm_sin_f32(PI*5/4+raw));
							wheelspeed[3]+=(int)((double)x_s*3*arm_sin_f32(PI*7/4+raw));
						
			pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
			yaw_ppiidd=PID_YAW(&pid_yaw);
			pid_yawv.SetSpeed=yaw_ppiidd;
			pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
			yaw_ppiiddv=PID_YAWV(&pid_yaw);
			
							
			for(i=0;i<4;i++){
				wheel[i].SetSpeed=0;

			}				
				wheel[0].SetSpeed=yaw_ppiiddv;
				wheel[1].SetSpeed=yaw_ppiiddv;
				wheel[2].SetSpeed=1.588*yaw_ppiiddv;
				wheel[3].SetSpeed=1.588*yaw_ppiiddv;
				
			
							
			for(i=0;i<4;i++){
				wheel[i].SetSpeed+=wheelspeed[i];
				wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
				elecurrent[i]=(int)PID_Calc(&wheel[i]);	//printf("elecurrent[1]=%d",elecurrent[1]);
				
			}
			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);		

	}
	else{
		PID_POS_start(pid_pos,wheel,3500-550.000000,5700.000000-240,ACTION_GL_POS_DATA.ANGLE_Z);
		Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
		//printf("i1=%d\r\n",i1);
	}		
	
	
}

float pos_speed_xy_last_speed=0;
int  Pos_speed_XY(float set_x,float set_y,float speed){
	float acutual_x=0,acutual_y=0,x=0,y=0,r=0,r_square=0,acc_v=3;
	
	
		acutual_y=ACTION_GL_POS_DATA.POS_Y;
		acutual_x=ACTION_GL_POS_DATA.POS_X;
	
//	acutual_y=(quad_s[0]+quad_s[1]+quad_s[2]+quad_s[3])/4;
//  	acutual_x=(quad_s[0]-quad_s[1]-quad_s[2]+quad_s[3])/4;
	
	
	x=(set_x-acutual_x);
	y=(set_y-acutual_y);
	r_square=x*x+y*y;
	r=(float)sqrt((double)r_square);
	
	if(((r_square/1000000)+20)<speed||((r_square/1000000)+20)<pos_speed_xy_last_speed){
		if(r<160){
			//pos_stop(set_x,set_y);//停止
			
			//PRINTF("%f,%f,%f\r\n",x,y,r);
			
//			if(r<80)
				return 1;
			
			
			//return 0;
		}
		
		
		speed=((r_square/1000000)+20);
//		if(speed<30)
//			speed=30;
		if(speed-pos_speed_xy_last_speed>acc_v){
		speed=pos_speed_xy_last_speed+acc_v;
	}
		pos_speed_xy_last_speed=speed;
		Speed_XY(x/r*speed,y/r*speed);
		//PRINTF("%f,%f,%f\r\n",x,y,r);
		return 0;
	}

	if(speed-pos_speed_xy_last_speed>acc_v){
		
		speed=pos_speed_xy_last_speed+acc_v;
		pos_speed_xy_last_speed=speed;
	}
	Speed_XY(x/r*speed,y/r*speed);
	//PRINTF("%f,%f,%f\r\n",x,y,r);
	return 0;
	
//	if(speed-pos_speed_xy_last_speed<-1)
//		speed-=1;
}

int Auto_lidar_XY(float x,float y){
 
 static float pid_vlr=0,lidar_x=0,lidar_y=0;
 static int a_l_num=0;
	
	 lidar_x=slave_X;
	 lidar_y=slave_Y;
   pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
   yaw_ppiidd=PID_YAW(&pid_yaw);
   pid_yawv.SetSpeed=yaw_ppiidd;
   pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
   yaw_ppiiddv=PID_YAWV(&pid_yawv);
   
       
   for(int i=0;i<4;i++){
    wheel0[i].SetSpeed=0;

   }    
    wheel0[0].SetSpeed=yaw_ppiiddv;
    wheel0[1].SetSpeed=yaw_ppiiddv;
    wheel0[2].SetSpeed=yaw_ppiiddv;
    wheel0[3].SetSpeed=yaw_ppiiddv;
   
   
  if((int)x){
  pid_Lidar_position.SetSpeed=x;
  pid_Lidar_position.AcutualSpeed=lidar_x;
  pid_vlr=-PID_Calc(&pid_Lidar_position);
   wheel0[0].SetSpeed-=pid_vlr;
   wheel0[1].SetSpeed-=pid_vlr;
   wheel0[2].SetSpeed+=pid_vlr;
   wheel0[3].SetSpeed+=pid_vlr;
	}
   if((int)y){
  pid_Lidar_position_H.SetSpeed=y;
  pid_Lidar_position_H.AcutualSpeed=lidar_y;
  pid_vlr=PID_Calc(&pid_Lidar_position_H);
   wheel0[0].SetSpeed-=pid_vlr;
   wheel0[1].SetSpeed+=pid_vlr;
   wheel0[2].SetSpeed+=pid_vlr;
   wheel0[3].SetSpeed-=pid_vlr; 
   }
//   printf("%f,%f,%f\r\n",pid_Lidar_position_H.SetSpeed,pid_Lidar_position_H.AcutualSpeed,pid_Lidar_position.AcutualSpeed);
   
  for(int i=0;i<4;i++){
 //  wheel0[i].SetSpeed+=wheelspeed[i];
   wheel0[i].AcutualSpeed=(float)Motor[i].NowSpeed;
   elecurrent[i]=(int)PID_Calc(&wheel0[i]); //printf("elecurrent[1]=%d",elecurrent[1]);
   
  }
  Send_Motor_current(0x200,CAN_Id_Standard,elecurrent); 
  

//				   if(fabs((double)(x-lidar_x))<3&&fabs((double)(y-lidar_y))<3)
//					 {
//							a_l_num++;
//							if(a_l_num>50)
//							{
//							 a_l_num=0;
//							 return 0;
//							}
//					 }

  
  
 return 1;
 
}

int Pos_XY(float x,float y){
	
	static float pid_vlr=0,acut_x=0,acut_y=0;
 static int a_l_num=0;
	
	
	 acut_x=ACTION_GL_POS_DATA.POS_X;
	 acut_y=ACTION_GL_POS_DATA.POS_Y;
   pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
   yaw_ppiidd=PID_YAW(&pid_yaw);
   pid_yawv.SetSpeed=yaw_ppiidd;
   pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
   yaw_ppiiddv=PID_YAWV(&pid_yawv);
   
       
   for(int i=0;i<4;i++){
    wheel0[i].SetSpeed=0;

   }    
    wheel0[0].SetSpeed=yaw_ppiiddv;
    wheel0[1].SetSpeed=yaw_ppiiddv;
    wheel0[2].SetSpeed=yaw_ppiiddv;
    wheel0[3].SetSpeed=yaw_ppiiddv;
   
   

  Pos_pidx.SetSpeed=x;
  Pos_pidx.AcutualSpeed=acut_x;
  pid_vlr=PID_Calc(&Pos_pidx);
   wheel0[0].SetSpeed+=pid_vlr;
   wheel0[1].SetSpeed+=pid_vlr;
   wheel0[2].SetSpeed-=pid_vlr;
   wheel0[3].SetSpeed-=pid_vlr;
	
  
  Pos_pidy.SetSpeed=y;
  Pos_pidy.AcutualSpeed=acut_y;
  pid_vlr=PID_Calc(&Pos_pidy);
   wheel0[0].SetSpeed+=pid_vlr;
   wheel0[1].SetSpeed-=pid_vlr;
   wheel0[2].SetSpeed-=pid_vlr;
   wheel0[3].SetSpeed+=pid_vlr; 
   
//   printf("%f,%f,%f\r\n",pid_Lidar_position_H.SetSpeed,pid_Lidar_position_H.AcutualSpeed,pid_Lidar_position.AcutualSpeed);
   
  for(int i=0;i<4;i++){
 //  wheel0[i].SetSpeed+=wheelspeed[i];
   wheel0[i].AcutualSpeed=(float)Motor[i].NowSpeed;
   elecurrent[i]=(int)PID_Calc(&wheel0[i]); //printf("elecurrent[1]=%d",elecurrent[1]);
   
  }
  Send_Motor_current(0x200,CAN_Id_Standard,elecurrent); 
  

				   if(fabs((double)(x-acut_x))<30&&fabs((double)(y-acut_y))<30)
					 {
							a_l_num++;
							if(a_l_num>5)
							{
							 a_l_num=0;
							 return 0;
							}
					 }

  
  
 return 1;
 
	
}

int Pos_XY1(float x1,float x,float y){
	
	static float pid_vlr=0,acut_x=0,acut_y=0,lad_x=0;
 static int a_l_num=0;
	
	
	 acut_x=ACTION_GL_POS_DATA.POS_X;
	 acut_y=ACTION_GL_POS_DATA.POS_Y;
	 lad_x =laser_Res[3]/1000;
   pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
   yaw_ppiidd=PID_YAW(&pid_yaw);
   pid_yawv.SetSpeed=yaw_ppiidd;
   pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
   yaw_ppiiddv=PID_YAWV(&pid_yawv);
   
       
   for(int i=0;i<4;i++){
    wheel0[i].SetSpeed=0;

   }    
    wheel0[0].SetSpeed=yaw_ppiiddv;
    wheel0[1].SetSpeed=yaw_ppiiddv;
    wheel0[2].SetSpeed=yaw_ppiiddv;
    wheel0[3].SetSpeed=yaw_ppiiddv;
   
   
//if(acut_y>1300-750)
//{
//  Pos_pidx.SetSpeed=x;
//  Pos_pidx.AcutualSpeed=acut_x;
//  pid_vlr=PID_Calc(&Pos_pidx);
//} 
//else 
//{
	Pos_pidx1.SetSpeed=x;
  Pos_pidx1.AcutualSpeed=laser_Res[3]/1000;
  pid_vlr=-PID_Calc(&Pos_pidx1);
//

   wheel0[0].SetSpeed+=pid_vlr;
   wheel0[1].SetSpeed+=pid_vlr;
   wheel0[2].SetSpeed-=pid_vlr;
   wheel0[3].SetSpeed-=pid_vlr;
	
  
  Pos_pidy.SetSpeed=y;
  Pos_pidy.AcutualSpeed=acut_y;
  pid_vlr=PID_Calc(&Pos_pidy);
   wheel0[0].SetSpeed+=pid_vlr;
   wheel0[1].SetSpeed-=pid_vlr;
   wheel0[2].SetSpeed-=pid_vlr;
   wheel0[3].SetSpeed+=pid_vlr; 
   
//   printf("%f,%f,%f\r\n",pid_Lidar_position_H.SetSpeed,pid_Lidar_position_H.AcutualSpeed,pid_Lidar_position.AcutualSpeed);
   
  for(int i=0;i<4;i++){

   wheel0[i].AcutualSpeed=(float)Motor[i].NowSpeed;
   elecurrent[i]=(int)PID_Calc(&wheel0[i]); //printf("elecurrent[1]=%d",elecurrent[1]);
   
  }
  Send_Motor_current(0x200,CAN_Id_Standard,elecurrent); 
  

				   if(fabs((double)(x-lad_x))<15&&fabs((double)(y-acut_y))<30)
					 {
							a_l_num++;
							if(a_l_num>5)
							{
							 a_l_num=0;
							 return 0;
							}
					 }

  
  
 return 1;
 
	
}


void speed_xyz(float v_x,float v_y,float v_w)
{
	float yaw_pid=0,yaw_ppiidd=0,yaw_ppiiddv=0,raw=0;
	raw=(double)ACTION_GL_POS_DATA.ANGLE_Z/180*PI;
	
	
	
	pid_yaw.SetSpeed=0;
	
//	pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
//	yaw_ppiidd=PID_YAW(&pid_yaw);
//	pid_yawv.SetSpeed=yaw_ppiidd;
//	pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
//	yaw_ppiiddv=PID_YAWV(&pid_yaw);
	
	
	for(int i=0;i<4;i++)
	{
		wheelspeed[i]=0;	
	}
						
		wheelspeed[0]+=(int)((double)v_y*arm_cos_f32(PI*1/4));
		wheelspeed[1]+=(int)((double)v_y*arm_cos_f32(PI*3/4));
		wheelspeed[2]+=(int)((double)v_y*arm_cos_f32(PI*5/4));
		wheelspeed[3]+=(int)((double)v_y*arm_cos_f32(PI*7/4));
	
	
		wheelspeed[0]+=(int)((double)v_x*arm_sin_f32(PI*1/4));
		wheelspeed[1]+=(int)((double)v_x*arm_sin_f32(PI*3/4));
		wheelspeed[2]+=(int)((double)v_x*arm_sin_f32(PI*5/4));
		wheelspeed[3]+=(int)((double)v_x*arm_sin_f32(PI*7/4));
							
		for(int i=0;i<4;i++)
	 {
			wheel[i].SetSpeed=0;
	 }				
			wheel[0].SetSpeed=v_w;
			wheel[1].SetSpeed=v_w;
			wheel[2].SetSpeed=v_w;
			wheel[3].SetSpeed=v_w;
			
		
						
		for(int i=0;i<4;i++)
	 {
			wheel[i].SetSpeed+=wheelspeed[i];
			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
			elecurrent[i]=(int)PID_Calc(&wheel[i]);	//printf("elecurrent[1]=%d",elecurrent[1]);
	 }
		Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);	
	
		}




//int a11=0,a22=0,a111=0,a222=0;
//double raw2=0;
//void Handle_control(){
//	int i=0,y=0,x=0,r=0;
//	double raw=0,k=6.772133;
//	x=rc_ctrl.rc.ch[2];
//	y=rc_ctrl.rc.ch[3];
//	r=rc_ctrl.rc.ch[0];
//	//raw=((Motor[0].SerialEnconder/10000+Motor[2].SerialEnconder/10000)+(Motor[1].SerialEnconder/10000+Motor[3].SerialEnconder/10000))*2*k/1000;
//	raw=(double)ACTION_GL_POS_DATA.ANGLE_Z/180*PI;
//	
//	for(i=0;i<4;i++){
//				wheelspeed[i]=0;	
//			}
//	if(y>30||y<-30){
//		wheelspeed[0]+=(int)((double)y*3*arm_cos_f32(PI*1/4+raw+raw2));
//		wheelspeed[1]+=(int)((double)y*3*arm_cos_f32(PI*3/4+raw+raw2));
//		wheelspeed[2]+=(int)((double)y*3*arm_cos_f32(PI*5/4+raw+raw2));
//		wheelspeed[3]+=(int)((double)y*3*arm_cos_f32(PI*7/4+raw+raw2));
//	}
//	if(x>30||x<-30){
//		wheelspeed[0]+=(int)((double)x*3*arm_sin_f32(PI*1/4+raw+raw2));
//		wheelspeed[1]+=(int)((double)x*3*arm_sin_f32(PI*3/4+raw+raw2));
//		wheelspeed[2]+=(int)((double)x*3*arm_sin_f32(PI*5/4+raw+raw2));
//		wheelspeed[3]+=(int)((double)x*3*arm_sin_f32(PI*7/4+raw+raw2));
//	}
//	if(r>30||r<-30){
//		wheelspeed[2]+=3*(int)r;
//		wheelspeed[3]+=3*(int)r;
//		wheelspeed[0]+=3*(int)r;
//		wheelspeed[1]+=3*(int)r;
//	}
//	

//	
//	if(rc_ctrl.rc.ch[1]>655)
//		a11=1;
//	if(rc_ctrl.rc.ch[1]< 655&&a11==1){
//		a11=0;
//		raw2-=PI/2;
//	}
//	if(rc_ctrl.rc.ch[1]<-655)
//		a22=1;
//	if(rc_ctrl.rc.ch[1]>-655&&a22==1){
//		a22=0;
//		raw2+=PI/2;
//	}
//	
//	for(i=0;i<4;i++){
//			wheel[i].SetSpeed=wheelspeed[i];
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			elecurrent[i]=(int)PID_Calc(&wheel[i]);	
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		}
//	
//}




//void Handle_control_Car(int speed){
//	int i=0,y=0,x=0,r=0;
//	double raw=0,k=6.772133,raw2=0;
//	x=rc_ctrl.rc.ch[2];
//	y=rc_ctrl.rc.ch[3];
//	r=rc_ctrl.rc.ch[0];
//	//raw=((Motor[0].SerialEnconder/10000+Motor[2].SerialEnconder/10000)+(Motor[1].SerialEnconder/10000+Motor[3].SerialEnconder/10000))*2*k/1000;
//	raw=(double)ACTION_GL_POS_DATA.ANGLE_Z/180*PI;
//	
//	for(i=0;i<4;i++){
//				wheelspeed[i]=0;	
//			}
//	if(y>30||y<-30){
//		wheelspeed[0]+=y*speed;
//		wheelspeed[1]-=y*speed;
//		wheelspeed[2]-=y*speed;
//		wheelspeed[3]+=y*speed;
//	}
//	if(x>30||x<-30){
//		wheelspeed[0]+=x*speed;
//		wheelspeed[1]+=x*speed;
//		wheelspeed[2]-=x*speed;
//		wheelspeed[3]-=x*speed;
//	}
//	if(r>30||r<-30){
//		wheelspeed[2]+=3*(int)r;
//		wheelspeed[3]+=3*(int)r;
//		wheelspeed[0]+=3*(int)r;
//		wheelspeed[1]+=3*(int)r;
//	}
//	
//	
//	for(i=0;i<4;i++){
//			wheel[i].SetSpeed=wheelspeed[i];
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			elecurrent[i]=(int)PID_Calc(&wheel[i]);	
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		}
//	
//}



//void Handle_control_Car_2(int speed){
//	int i=0,y=0,x=0,r=0;
//	double raw=0,k=6.772133,raw2=0;
//	x=rc_ctrl.rc.ch[2]*rc_ctrl.rc.ch[2]/660;
//	y=rc_ctrl.rc.ch[3]*rc_ctrl.rc.ch[3]/660;
//	r=rc_ctrl.rc.ch[0]*rc_ctrl.rc.ch[0]/660;
//	if(rc_ctrl.rc.ch[3]<0)
//		y=-y;
//	if(rc_ctrl.rc.ch[2]<0)
//		x=-x;
//	if(rc_ctrl.rc.ch[0]<0)
//		r=-r;	
//	//raw=((Motor[0].SerialEnconder/10000+Motor[2].SerialEnconder/10000)+(Motor[1].SerialEnconder/10000+Motor[3].SerialEnconder/10000))*2*k/1000;
//	raw=(double)ACTION_GL_POS_DATA.ANGLE_Z/180*PI;
//	
//	for(i=0;i<4;i++){
//				wheelspeed[i]=0;	
//			}
//	if(y>30||y<-30){
//		wheelspeed[0]+=y*speed;
//		wheelspeed[1]-=y*speed;
//		wheelspeed[2]-=y*speed;
//		wheelspeed[3]+=y*speed;
//	}
//	if(x>30||x<-30){
//		wheelspeed[0]+=x*speed;
//		wheelspeed[1]+=x*speed;
//		wheelspeed[2]-=x*speed;
//		wheelspeed[3]-=x*speed;
//	}
//	if(r>30||r<-30){
//		wheelspeed[2]+=3*(int)r;
//		wheelspeed[3]+=3*(int)r;
//		wheelspeed[0]+=3*(int)r;
//		wheelspeed[1]+=3*(int)r;
//	}
//	
//	
//	for(i=0;i<4;i++){
//			wheel[i].SetSpeed=wheelspeed[i];
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			elecurrent[i]=(int)PID_Calc(&wheel[i]);	
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		}
//	
//}





//void Handle_control_Car_lidar(int speed){
//	int i=0,y=0,x=0,r=0;
//	double raw=0,k=6.772133,raw2=0;
//	x=rc_ctrl.rc.ch[2];
//	y=rc_ctrl.rc.ch[3];
//	r=rc_ctrl.rc.ch[0];
//	//raw=((Motor[0].SerialEnconder/10000+Motor[2].SerialEnconder/10000)+(Motor[1].SerialEnconder/10000+Motor[3].SerialEnconder/10000))*2*k/1000;
//	raw=(double)ACTION_GL_POS_DATA.ANGLE_Z/180*PI;
//	
//	for(i=0;i<4;i++){
//				wheelspeed[i]=0;	
//			}
//	if(y>30||y<-30){
//		wheelspeed[0]+=y*speed;
//		wheelspeed[1]-=y*speed;
//		wheelspeed[2]-=y*speed;
//		wheelspeed[3]+=y*speed;
//	}
//	if(x>30||x<-30){
//		wheelspeed[0]+=x*speed;
//		wheelspeed[1]+=x*speed;
//		wheelspeed[2]-=x*speed;
//		wheelspeed[3]-=x*speed;
//	}
////	if(r>30||r<-30){
////		wheelspeed[2]+=3*(int)r;
////		wheelspeed[3]+=3*(int)r;
////		wheelspeed[0]+=3*(int)r;
////		wheelspeed[1]+=3*(int)r;
////	}
//	if(rc_ctrl.rc.ch[1]==660)
//		a11=1;
//	if(rc_ctrl.rc.ch[1]!=660&&a11==1){
//		a11=0;
//		raw2-=PI/2;
//	}
//	if(rc_ctrl.rc.ch[1]==-660)
//		a22=1;
//	if(rc_ctrl.rc.ch[1]!=-660&&a22==1){
//		a22=0;
//		raw2+=PI/2;
//	}
//	
//	
//	pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
//			yaw_ppiidd=PID_YAW(&pid_yaw);
//			pid_yawv.SetSpeed=yaw_ppiidd;
//			pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
//			yaw_ppiiddv=PID_YAWV(&pid_yaw);
//			
//							
//			for(i=0;i<4;i++){
//				wheel[i].SetSpeed=0;

//			}				
//				wheel[0].SetSpeed=yaw_ppiiddv;
//				wheel[1].SetSpeed=yaw_ppiiddv;
//				wheel[2].SetSpeed=1.588*yaw_ppiiddv;
//				wheel[3].SetSpeed=1.588*yaw_ppiiddv;
//	
//	for(i=0;i<4;i++){
//			wheel[i].SetSpeed+=wheelspeed[i];
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			elecurrent[i]=(int)PID_Calc(&wheel[i]);	
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		}
//	
//}

//void Handle_control_UP(){
//	int i=0,y=0,x=0,r=0,l=0;
//	double raw=0,k=6.772133,raw2=0;
//	x=rc_ctrl2.rc.ch[2];
//	y=rc_ctrl2.rc.ch[3];
//	l=rc_ctrl2.rc.ch[1];
//	r=rc_ctrl2.rc.ch[0];
//	//printf("%d %d %d\r\n",x,y,r);
//	if(y>30||y<-30){
//		pid_HL.SetSpeed=(3*(float)y);
//		if((float)(Motor[7].SerialEnconder*19/20000)<0&&y<0)
//			pid_HL.SetSpeed=0;
//		pid_HL.AcutualSpeed=(float)Motor[7].NowSpeed;
//		elecurrent2[3]=PID_Calc(&pid_HL);
//		H_L=(float)(Motor[7].SerialEnconder*19/20000);
//	}else{
//		pid_HL_position.SetSpeed=H_L;
//		pid_HL.AcutualSpeed=(float)Motor[7].NowSpeed;
//		pid_HL_position.AcutualSpeed=(float)(Motor[7].SerialEnconder*19/20000);
//		elecurrent2[3]=PID_Incre(&pid_HL_position,&pid_HL);
//	}
////	if(x>30||x<-30){
////		
////	}
//	//printf("%d,%d\r\n",x,a11);
//	if(l>655){
//		a111=1;
//	}
//	if(l<655&&a111==1){
//		a111=0;
//		pid_claw_position[0].SetSpeed+=1751*2/5;
//		pid_claw_position[1].SetSpeed-=1751*2/5;
//		claw_sign+=1;
//	}

//	if(l<-655)
//		a222=1;
//	if(l>-655&&a222==1){
//		a222=0;
//		pid_claw_position[0].SetSpeed-=1751*2/5;
//		pid_claw_position[1].SetSpeed+=1751*2/5;
//		claw_sign+=1;
//	}
//	if(r>30||r<-30){
//		pid_LR.SetSpeed=(3*(float)r);
//		pid_LR.AcutualSpeed=(float)Motor[6].NowSpeed;
//		elecurrent2[2]=PID_Calc(&pid_LR);
//		L_R=(float)(Motor[6].SerialEnconder*19/20000);
//	}else{
//		pid_LR_position.SetSpeed=L_R;
//		pid_LR.AcutualSpeed=(float)Motor[6].NowSpeed;
//		pid_LR_position.AcutualSpeed=(float)(Motor[6].SerialEnconder*19/20000);
//		elecurrent2[2]=PID_Incre(&pid_LR_position,&pid_LR);
//	}

////高低
//		
//		


////左右
//		
//		
//		
////爪子
//		pid_claw_position[0].AcutualSpeed=(float)(Motor[4].SerialEnconder*19/2000);
//		pid_claw[0].SetSpeed=PID_Calc(&pid_claw_position[0]);
//		pid_claw[0].AcutualSpeed=(float)Motor[4].NowSpeed;
//		elecurrent2[0]=PID_Calc(&pid_claw[0]);
//		
//		
//		pid_claw_position[1].AcutualSpeed=(float)(Motor[5].SerialEnconder*19/2000);
//		pid_claw[1].SetSpeed=PID_Calc(&pid_claw_position[1]);
//		pid_claw[1].AcutualSpeed=(float)Motor[5].NowSpeed;
//		elecurrent2[1]=PID_Calc(&pid_claw[1]);
//		
//		
//		Send_Motor_current(0x1FF,CAN_Id_Standard,elecurrent2);

//}
//void Handle_control_UP_Ready(){
//			pid_LR_position.SetSpeed=L_R;
//			pid_LR.AcutualSpeed=(float)Motor[6].NowSpeed;
//			pid_LR_position.AcutualSpeed=(float)(Motor[6].SerialEnconder*19/20000);
//			elecurrent2[2]=PID_Incre(&pid_LR_position,&pid_LR);
//		
//			pid_HL_position.SetSpeed=H_L;
//			pid_HL.AcutualSpeed=(float)Motor[7].NowSpeed;
//			pid_HL_position.AcutualSpeed=(float)(Motor[7].SerialEnconder*19/20000);
//			elecurrent2[3]=PID_Incre(&pid_HL_position,&pid_HL);
//		
//		
//			pid_claw_position[0].AcutualSpeed=(float)(Motor[4].SerialEnconder*19/2000);
//			pid_claw[0].SetSpeed=PID_Calc(&pid_claw_position[0]);
//			pid_claw[0].AcutualSpeed=(float)Motor[4].NowSpeed;
//			elecurrent2[0]=PID_Calc(&pid_claw[0]);
//			
//			
//			pid_claw_position[1].AcutualSpeed=(float)(Motor[5].SerialEnconder*19/2000);
//			pid_claw[1].SetSpeed=PID_Calc(&pid_claw_position[1]);
//			pid_claw[1].AcutualSpeed=(float)Motor[5].NowSpeed;
//			elecurrent2[1]=PID_Calc(&pid_claw[1]);
//			
//			Send_Motor_current(0x1FF,CAN_Id_Standard,elecurrent2);
//}


//float back_yaw;
////调用前记录角度,记录位置
//void Back_blocks(int *sign,float x,float y){
//	float l=0;
//	switch (*sign){
//		case 0:{
//			
//			
//		}break;
//		case 1:{
//			l=0;
//		}break;
//		case 2:{
//			
//				
//			l=37;
//		}break;
//		case 3:{
//			    
//			l=75;
//		}break;
//		case 4:{
//			l=112;
//			
//		}break;
//		case 5:{
//			l=150;
//			
//		}break;
//		default:{
//				return;
//		}break;
//	}
//			
//				//	printf("%f\r\n",l);
//			PID_POS_start(pid_pos,wheel,y-l*(float)arm_cos_f32((double)pid_yaw.SetSpeed),x-l*(float)arm_sin_f32((double)pid_yaw.SetSpeed),ACTION_GL_POS_DATA.ANGLE_Z);
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);		
//	
//	
//	
//}

//void  Claw_pos(float lr)  {
//		pid_LR.SetSpeed=lr;
//		pid_LR.AcutualSpeed=(float)Motor[6].NowSpeed;
//		pid_LR_position.AcutualSpeed=(float)(Motor[6].SerialEnconder*19/20000);
//		elecurrent2[2]=PID_Incre(&pid_LR_position,&pid_LR);
//		Send_Motor_current(0x1FF,CAN_Id_Standard,elecurrent2);
//}
//int clip_sss=1;
//void Clip_blocks(int *sign){
//	switch (*sign){
//		case 0:{
//			Handle_control_UP_Ready();
//			//return;
//			
//		}break;
//		case 1:{
//				L_R=-55;
//				Handle_control_UP_Ready();		
//		}break;
//		case 2:{
//			L_R=-100;
//				Handle_control_UP_Ready();
//			
//		}break;
//		case 3:{
//			L_R=-145;
//				Handle_control_UP_Ready();
//		}break;
//		case 4:{
//			
//			L_R=0;
//			Handle_control_UP_Ready();
//		}break;
//		case 7:{
//			L_R=-190;
//				Handle_control_UP_Ready();
//			
//		}break;
//		case 5:{
//			L_R=-235;
//				Handle_control_UP_Ready();
//			
//		}break;
//		case 6:{
//		
//			Handle_control_UP_Ready();
//		}break;
//		
//		case 8:{
//		
//			Handle_control_UP_Ready();
//		}break;
//		
//		default:{
//				Handle_control_UP_Ready();
//		}break;
//		
//		
//	}
//	
//}

//int clip_blocks_new_sign=0;
//void Clip_blocks_new() {
//	printf("%d\r\n",elecurrent2[2]);
//	if(Motor[6].NowSpeed<-2200){
//		clip_blocks_new_sign=1;
//	}
//	if(clip_blocks_new_sign==1&&Motor[6].NowSpeed>-1400){
//		Handle_control_UP_Ready();
//	}else{
//		
//			L_R=(float)(Motor[6].SerialEnconder*19/20000);
//			pid_LR.SetSpeed=-8000;
//			pid_LR.AcutualSpeed=(float)Motor[6].NowSpeed;
//			elecurrent2[2]=PID_Calc(&pid_LR);//PID_Incre(&pid_LR_position,);
//			
//		
//		
//		
//	
//		
//			pid_HL_position.SetSpeed=H_L;
//			pid_HL.AcutualSpeed=(float)Motor[7].NowSpeed;
//			pid_HL_position.AcutualSpeed=(float)(Motor[7].SerialEnconder*19/20000);
//			elecurrent2[3]=PID_Incre(&pid_HL_position,&pid_HL);
//		
//		
//			pid_claw_position[0].AcutualSpeed=(float)(Motor[4].SerialEnconder*19/2000);
//			pid_claw[0].SetSpeed=PID_Calc(&pid_claw_position[0]);
//			pid_claw[0].AcutualSpeed=(float)Motor[4].NowSpeed;
//			elecurrent2[0]=PID_Calc(&pid_claw[0]);
//			
//			
//			pid_claw_position[1].AcutualSpeed=(float)(Motor[5].SerialEnconder*19/2000);
//			pid_claw[1].SetSpeed=PID_Calc(&pid_claw_position[1]);
//			pid_claw[1].AcutualSpeed=(float)Motor[5].NowSpeed;
//			elecurrent2[1]=PID_Calc(&pid_claw[1]);
//			
//			Send_Motor_current(0x1FF,CAN_Id_Standard,elecurrent2);
//		
//	}
//	
//}

//void Clip_blocks2(int *sign){
//	switch (*sign){
//		case 0:{
//			Handle_control_UP_Ready();
//			//return;
//			
//		}break;
//		case 1:{
//				H_L=387;
//				Handle_control_UP_Ready();	
//		}break;
//		case 2:{
//				H_L=562;
//				Handle_control_UP_Ready();	
//		}break;
//		case 3:{
//				H_L=746;
//				Handle_control_UP_Ready();	
//		}break;
//		case 4:{
//				H_L=0;
//			Handle_control_UP_Ready();
//		}break;
//		
//		case 7:{
//				H_L=924;
//				Handle_control_UP_Ready();	
//		}break;
//		case 5:{
//				H_L=1050;
//				Handle_control_UP_Ready();
//		}break;
//		case 6:{
//		
//			Handle_control_UP_Ready();
//		}break;
//		case 8:{
//		
//			Handle_control_UP_Ready();
//		}break;
//		default:{
//				Handle_control_UP_Ready();
//		}break;
//		
//		
//	}
//	
//}

//int clip_od_sign=0;
//void Clip_open_Down(int *sign){
//		L_R=0;
//		switch (*sign){
//		case 0:{
//			Handle_control_UP_Ready();
//			//return;
//			
//		}break;
//		case 1:{
//				H_L=0;
//				if(claw_sign%2!=0){
//					pid_claw_position[0].SetSpeed-=1751*2/5;
//					pid_claw_position[1].SetSpeed+=1751*2/5;
//					claw_sign+=1;
//				}
//				Handle_control_UP_Ready();	
//		}break;
//		case 2:{
//			Handle_control_UP_Ready();
//		}break;
//		case 3:{
//			Handle_control_UP_Ready();
//		}break;
//		case 4:{
//			Handle_control_UP_Ready();
//		}break;
//		case 5:{
//			Handle_control_UP_Ready();
//		}break;
//		
//		
//	}
//	
//}



  //printf("%d,%d,%d,%d\r\n",Motor[0].NowSpeed,Motor[1].NowSpeed,Motor[2].NowSpeed,Motor[3].NowSpeed);
		//printf("%d,%d,%d,%d\r\n",Motor[0].NowEnconder,Motor[1].NowEnconder,Motor[2].NowEnconder,Motor[3].NowEnconder);
		//printf("%d,%d,%d,%d\r\n",(Motor[0].SerialEnconder*19/20000),(Motor[1].SerialEnconder*19/20000),(Motor[2].SerialEnconder*19/20000),(Motor[3].SerialEnconder*19/20000));
		//printf("%d,%d,%d,%d,%d,%d\r\n",rc_ctrl.rc.ch[0],rc_ctrl.rc.ch[1],rc_ctrl.rc.ch[2],rc_ctrl.rc.ch[3],rc_ctrl.rc.s[0],rc_ctrl.rc.s[1]);
		//printf("%f,%f,%f,%f",wheel[0].SetSpeed,wheel[0].AcutualSpeed,wheel[0].err,(int)elecurrent[0]);
		//printf("%d\r\n",Motor[2].NowSpeed);
		//printf("%f,,%f,,%f,,%f\r\n",ACTION_GL_POS_DATA.W_Z,yaw_ppiiddv,ACTION_GL_POS_DATA.ANGLE_Z,ACTION_GL_POS_DATA.POS_X,ACTION_GL_POS_DATA.POS_Y);
		//t=(Motor[0].SerialEnconder/1000+Motor[2].SerialEnconder/1000)+(Motor[1].SerialEnconder/1000+Motor[3].SerialEnconder/1000);
		//printf("%d\r\n",t);//4639000*x=pi
//		for(i=0;i<4;i++){
//			wheel[i].SetSpeed=wheelspeed[i];
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			elecurrent[i]=(int)PID_Calc(&wheel[i]);	

//		}
		
//	位置	
//		for(i=0;i<4;i++){
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			pid_position[i].SetSpeed=wheelposition[i];
//			pid_position[i].AcutualSpeed=(float)(Motor[i].SerialEnconder*19/20000);
//			elecurrent[i]=PID_Incre(&pid_position[i],&wheel[i]);
//	
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		
//		
//		}

//速度
//		for(i=0;i<4;i++){
//			wheel[i].SetSpeed=wheelspeed[i];
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			elecurrent[i]=(int)PID_Calc(&wheel[i]);	
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		}

		
//	pos位置	

//		for(i=0;i<4;i++){
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			pid_position[i].SetSpeed=wheelposition[i];
//			pid_position[i].AcutualSpeed=(float)(Motor[i].SerialEnconder*19/20000);
//			elecurrent[i]=PID_Incre(&pid_position[i],&wheel[i]);
//	
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		
//		
//		}


//角度环
//			pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
//			yaw_ppiidd=PID_YAW(&pid_yaw);
//			pid_yawv.SetSpeed=yaw_ppiidd;
//			pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
//			yaw_ppiiddv=PID_YAWV(&pid_yaw);
//			for(i=0;i<4;i++){
//				wheel[i].SetSpeed=yaw_ppiiddv;
//				wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//				elecurrent[i]=(int)PID_Calc(&wheel[i]);	
//			}
//			Send_

//pos+yaw
//PID_POS_start(pid_pos,wheel,x_pos,y_pos,ACTION_GL_POS_DATA.ANGLE_Z);
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);


//高低
//		pid_HL.AcutualSpeed=(float)Motor[7].NowSpeed;
//		pid_HL_position.AcutualSpeed=(float)(Motor[7].SerialEnconder*19/20000);
//		elecurrent2[3]=PID_Incre(&pid_HL_position,&pid_HL);
//		Send_Motor_current(0x1FF,CAN_Id_Standard,elecurrent2);


//左右
//		pid_LR.AcutualSpeed=(float)Motor[6].NowSpeed;
//		pid_LR_position.AcutualSpeed=(float)(Motor[6].SerialEnconder*19/20000);
//		elecurrent2[2]=PID_Incre(&pid_LR_position,&pid_LR);
//		Send_Motor_current(0x1FF,CAN_Id_Standard,elecurrent2);



////爪子
//		pid_claw_position[0].AcutualSpeed=(float)(Motor[4].SerialEnconder*19/2000);
//		pid_claw[0].SetSpeed=PID_Calc(&pid_claw_position[0]);
//		pid_claw[0].AcutualSpeed=(float)Motor[4].NowSpeed;
//		elecurrent2[0]=PID_Calc(&pid_claw[0]);
//		Send_Motor_current(0x1FF,CAN_Id_Standard,elecurrent2);





//	if(pos_start_sign==1){
////		if(handel_sign==1)
////			Pos_control(&posxy1[2],30,400.0);
////		if(handel_sign==2)
////			Handle_control();
////	    printf("%f,,%f,,%f,,%f,,\r\n",ACTION_GL_POS_DATA.W_Z,ACTION_GL_POS_DATA.ANGLE_Z,ACTION_GL_POS_DATA.POS_X,ACTION_GL_POS_DATA.POS_Y);
////	}

////		elecurrent[0]=500;
////		elecurrent[1]=500;
////		elecurrent[2]=500;
////		elecurrent[3]=500;
////		
////		Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		
//		//pid_claw[0].SetSpeed=3000;
//		//pid_claw_position[0].SetSpeed=0;
//		
//		
////		pid_claw_position[0].AcutualSpeed=(float)(Motor[4].SerialEnconder*19/2000);
////		pid_claw[0].SetSpeed=PID_Calc(&pid_claw_position[0]);
////		pid_claw[0].AcutualSpeed=(float)Motor[4].NowSpeed;
////		elecurrent2[0]=PID_Calc(&pid_claw[0]);
//		
//		
////		elecurrent2[0]=1000;
////		elecurrent2[1]=1000;
////		elecurrent2[2]=1000;
//		elecurrent2[3]=3000;
//		//Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//		
//		
////		
////		pid_LR_position.SetSpeed=0;


////		pid_LR.AcutualSpeed=(float)Motor[6].NowSpeed;
////		pid_LR_position.AcutualSpeed=(float)(Motor[6].SerialEnconder*19/20000);
////		elecurrent2[2]=PID_Incre(&pid_LR_position,&pid_LR);
//		//elecurrent2[2]=PID_Calc(&pid_LR);
//		
//		
//		
////		Send_Motor_current(0x200,CAN_Id_Standard,elecurrent2);
////		printf("%d\r\n",Motor[6].NowSpeed);
//		printf("%d,%d\r\n",Motor[7].NowSpeed,Motor[7].SerialEnconder);
//		//printf("%d,%d\r\n",Motor[4].NowSpeed,Motor[4].SerialEnconder*19/2000,Motor[6].NowSpeed,Motor[7].NowSpeed);


//激光环
//printf("%f,%f,%d,%f,%f\r\n",pid_Lidar.AcutualSpeed,(float)ACTION_GL_POS_DATA.W_Z,Motor[0].NowSpeed,pid_yawv.SetSpeed,wheel[1].SetSpeed);
//		pid_Lidar.SetSpeed=0;
//		pid_Lidar.AcutualSpeed=-(laser_Res[2]/1000-laser_Res[1]/1000-40);
//		//pid_yaw.SetSpeed=(int)PID_Calc(&pid_Lidar);	

//			//pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
//			pid_yawv.SetSpeed=PID_Calc(&pid_Lidar);	
//			pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
//			vvv=PID_YAWV(&pid_yawv);
////			for(i=0;i<4;i++)
////			=PID_YAWV(&pid_yaw);
//			
//			for(i=0;i<4;i++){
//			wheel[i].SetSpeed=vvv;
//			wheel[i].AcutualSpeed=(float)Motor[i].NowSpeed;
//			elecurrent[i]=(int)PID_Calc(&wheel[i]);	
//			Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
//			}
