#include "PID.h"
#include "motor.h"
#include "math.h"

float y1=0,x1=0;
_pid stPID_Angle;	//舵轮角度环


//extern _pid pid_yaw;
//extern _pid pid_yawv;

//extern _pid pid_position[4];


float yaw_o=0;


void PID_Init(void)//PID 始值化
{
	PID_Param_Init(&(Motor_Endercode[0].pid_speed),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[0].pid_speed));
	
	PID_Param_Init(&(Motor_Endercode[0].pid_loc),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[0].pid_loc));
	
	PID_Param_Init(&(Motor_Endercode[1].pid_speed),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[1].pid_speed));
	
	PID_Param_Init(&(Motor_Endercode[1].pid_loc),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[1].pid_loc));
	
	PID_Param_Init(&(Motor_Endercode[2].pid_speed),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[2].pid_speed));
	
	PID_Param_Init(&(Motor_Endercode[2].pid_loc),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[2].pid_loc));
	
	PID_Param_Init(&(Motor[3].pid_loc),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor[3].pid_loc));
	
	PID_Param_Init(&(Motor[3].pid_speed),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor[3].pid_speed));
	
	PID_Param_Init(&stPID_Angle,PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&stPID_Angle);

}

void PID_Data_Init(_pid *pid)	//PID 数据始值化
{	
	pid->SetSpeed			=		0.0;		
	pid->AcutualSpeed	=		0.0;
	pid->err					=		0.0;
	pid->err_last			=		0.0;
	pid->interal			=		0.0;
	pid->voltage			=		0.0;
}


void PID_Param_Init(_pid *p,float IM,float VM)  //PID参数嵌入
{
	p->I_Max	=		IM;
	p->V_Max	=		VM;
}

float PID_Calc(_pid *pid)								
{
	
	
         //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	pid->interal	+=	pid->Ki * pid->err;									 //定义积分值累加差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
					pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
					pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值

	pid->voltage	=		pid->Kp * pid->err + pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
			 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	//pid->AcutualSpeed		=			pid->SetSpeed;   
	//pid->err_last	=		pid->err;  
	return pid->voltage;                                          //返回定义电压值
}

float PID_Incre(_pid *pid,_pid *wheel)
{
	       //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	pid->interal	+=	pid->Ki * pid->err;									 //定义积分值累加差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
					pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
					pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值

	pid->voltage	=		pid->Kp * pid->err + pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
			 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	//pid->AcutualSpeed		=			pid->SetSpeed;   
	//pid->err_last	=		pid->err;  
	
	
	wheel->SetSpeed=pid->voltage; 
	return PID_Calc(wheel);                                         	  //返回定义电压值
}


float PID_Incre_2(_pid *pid,_pid *wheel)
{
	       //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	pid->interal	+=	pid->Ki * pid->err;									 //定义积分值累加差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
					pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
					pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值

	pid->voltage	=		pid->Kp * pid->err + pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
			 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	//pid->AcutualSpeed		=			pid->SetSpeed;   
	//pid->err_last	=		pid->err;  
	
//	if(pid->voltage-wheel->SetSpeed>20)
//		wheel->SetSpeed+=20;
//	else if((pid->voltage-wheel->SetSpeed)<-20)
//		wheel->SetSpeed-=20;
//	else	
	wheel->SetSpeed=pid->voltage+yaw_ppiiddv; 
	return PID_Calc(wheel);                                         	  //返回定义电压值
}

float PID_Incre3(_pid *pid)
{
	       //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	pid->interal	+=	pid->Ki * pid->err;									 //定义积分值累加差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
					pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
					pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值

	pid->voltage	=		pid->Kp * pid->err + pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
			 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	//pid->AcutualSpeed		=			pid->SetSpeed;   
	//pid->err_last	=		pid->err;  
	
	
	
	return pid->voltage;                                         	  //返回定义电压值
}


float PID_POSS(_pid *pid,_pid *pidspeed,_pid *pidspeed2,int a)
{
	
	
	
	       //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	pid->interal	+=	pid->Ki * pid->err;									 //定义积分值累加差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
					pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
					pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值

	pid->voltage	=		pid->Kp * pid->err + pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
			 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	//pid->AcutualSpeed		=			pid->SetSpeed;   
	//pid->err_last	=		pid->err;  
	
	
//	pidspeed->SetSpeed=pid->voltage; 
//	pidspeed2->SetSpeed=-pid->voltage; 
	
	
			pid_position[a].SetSpeed=pid->voltage; //1000;
			pid_position[a].AcutualSpeed=(float)(Motor[a].SerialEnconder*19/20000);
			elecurrent[a]=PID_Incre3(&pid_position[a]);
	
	pidspeed->SetSpeed=pid->voltage+yaw_ppiiddv;; 
	pidspeed2->SetSpeed=-pid->voltage+yaw_ppiiddv;; 
			
	
	elecurrent[a]=(int)PID_Calc(pidspeed);
	elecurrent[a+2]=(int)PID_Calc(pidspeed2);
	
	
	return 1;                                         	  //返回定义电压值
}
int num=0;
int PID_POS_start(_pid *pid_position,_pid *pidspeed,float x,float y,float yaw)
{

			pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
			yaw_ppiidd=PID_YAW(&pid_yaw);
			pid_yawv.SetSpeed=yaw_ppiidd;
			pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
			yaw_ppiiddv=PID_YAWV(&pid_yaw);
	
//	pid_yaw.SetSpeed=0;
//	pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
//	yaw_o=PID_YAW(&pid_yaw);
	//yaw_o=0;
	x=x-ACTION_GL_POS_DATA.POS_X;
	y=y-ACTION_GL_POS_DATA.POS_Y;
	y1=y*arm_cos_f32(PI*((double)yaw)/180)+x*arm_sin_f32(PI*((double)yaw)/180);
	x1=x*arm_cos_f32(PI*((double)yaw)/180)-y*arm_sin_f32(PI*((double)yaw)/180);
	
	
			pidspeed[0].AcutualSpeed=(float)Motor[0].NowSpeed;
	    pidspeed[2].AcutualSpeed=(float)Motor[2].NowSpeed;
			pid_position[0].SetSpeed=x1+y1;
			pid_position[0].AcutualSpeed=0;
			PID_POSS(&pid_position[0],&pidspeed[0],&pidspeed[2],0);
	
			//Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
			pidspeed[1].AcutualSpeed=(float)Motor[1].NowSpeed;
	    pidspeed[3].AcutualSpeed=(float)Motor[3].NowSpeed;
			pid_position[1].SetSpeed=x1-y1;
			pid_position[1].AcutualSpeed=0;
			PID_POSS(&pid_position[1],&pidspeed[1],&pidspeed[3],1);

	 if(fabs(x)<10&&fabs(y)<10)
	{
		 		num++;
				if(num>2)
				{
				   num=0;
					 return 0;
				}
	
	}
	
						 return 1;
}
void PID_Camera(_pid *pid_position,_pid *pidspeed,float x,float y,float yaw)
{

			pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
			yaw_ppiidd=PID_YAW(&pid_yaw);
			pid_yawv.SetSpeed=yaw_ppiidd;
			pid_yawv.AcutualSpeed=(float)ACTION_GL_POS_DATA.W_Z;
			yaw_ppiiddv=PID_YAWV(&pid_yaw);
	
//	pid_yaw.SetSpeed=0;
//	pid_yaw.AcutualSpeed=(float)ACTION_GL_POS_DATA.ANGLE_Z;
//	yaw_o=PID_YAW(&pid_yaw);
	//yaw_o=0;
	x=x-(float)slave_X;
	y=(float)slave_Y-y;
	y1=y+x;
	x1=x-y;
	
	
			pidspeed[0].AcutualSpeed=(float)Motor[0].NowSpeed;
	    pidspeed[2].AcutualSpeed=(float)Motor[2].NowSpeed;
			pid_position[0].SetSpeed=x1+y1;
			pid_position[0].AcutualSpeed=0;
			PID_POSS(&pid_position[0],&pidspeed[0],&pidspeed[2],0);
	
			//Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
			pidspeed[1].AcutualSpeed=(float)Motor[1].NowSpeed;
	    pidspeed[3].AcutualSpeed=(float)Motor[3].NowSpeed;
			pid_position[1].SetSpeed=x1-y1;
			pid_position[1].AcutualSpeed=0;
			PID_POSS(&pid_position[1],&pidspeed[1],&pidspeed[3],1);

//	 if(fabs(x)<20&&fabs(y)<20)
//	{
//		 		num++;
//				if(num>2)
//				{
//				   num=0;
//					 return 0;
//				}
//	
//	}
//	
//						 return 1;
}
float PID_YAW(_pid *pid)								
{
	
	
         //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	pid->interal	+=	pid->Ki * pid->err;									 //定义积分值累加差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
					pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
					pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值

	pid->voltage	=		pid->Kp * pid->err + pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
			 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	//pid->AcutualSpeed		=			pid->SetSpeed;   
	//pid->err_last	=		pid->err;  
	return pid->voltage;                                          //返回定义电压值
}

float PID_YAWV(_pid *pid)								
{
	
	
         //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	pid->interal	+=	pid->Ki * pid->err;									 //定义积分值累加差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
					pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
					pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值

	pid->voltage	=		pid->Kp * pid->err + pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
			 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	//pid->AcutualSpeed		=			pid->SetSpeed;   
	//pid->err_last	=		pid->err;  
	return pid->voltage;                                          //返回定义电压值
}
	


