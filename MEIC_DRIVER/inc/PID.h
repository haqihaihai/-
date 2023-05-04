#ifndef __PID_H
#define __PID_H

#define PID_ANGLE_IMAX		8200  //积分上限
#define PID_ANGLE_VMAX		8200  //输出上限

typedef struct _PID{
	float Kp;                 //比例
	float Ki;                 //积分
	float Kd;                 //微分
	float SetSpeed;						//设定值
	float AcutualSpeed;				//实际值
	float err;								//偏差值
	float err_last;						//上一个偏差值
	float err_l_last;         //上上一个偏差值
	float I_Max;							//积分上限
	float V_Max;							//输出上限
	float voltage;						//定义电压值（控制执行器的变量）
	float interal;						//定义积分值
}_pid;

extern _pid stPID_Angle;

void PID_Init(void);																												                    //PID初始化
void PID_Data_Init(_pid *pid);																							                    //PID数据初值化
void PID_Param_Init(_pid *p,float IM,float VM);	//PID参数嵌入
float PID_Calc(_pid *pid);								
float PID_Incre(_pid *pid,_pid *pidcalc);         //增量式PID
float PID_Incre_2(_pid *pid,_pid *wheel);
float PID_POSS(_pid *pid,_pid *pidspeed,_pid *pidspeed2,int a);
int PID_POS_start(_pid *pid_position,_pid *pidspeed,float x,float y,float yaw);
void PID_Camera(_pid *pid_position,_pid *pidspeed,float x,float y,float yaw);
float PID_YAW(_pid *pid);		
float PID_YAWV(_pid *pid);
float PID_Incre3(_pid *pid);
#endif

