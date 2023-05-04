#ifndef __PID_H
#define __PID_H

#define PID_ANGLE_IMAX		8200  //��������
#define PID_ANGLE_VMAX		8200  //�������

typedef struct _PID{
	float Kp;                 //����
	float Ki;                 //����
	float Kd;                 //΢��
	float SetSpeed;						//�趨ֵ
	float AcutualSpeed;				//ʵ��ֵ
	float err;								//ƫ��ֵ
	float err_last;						//��һ��ƫ��ֵ
	float err_l_last;         //����һ��ƫ��ֵ
	float I_Max;							//��������
	float V_Max;							//�������
	float voltage;						//�����ѹֵ������ִ�����ı�����
	float interal;						//�������ֵ
}_pid;

extern _pid stPID_Angle;

void PID_Init(void);																												                    //PID��ʼ��
void PID_Data_Init(_pid *pid);																							                    //PID���ݳ�ֵ��
void PID_Param_Init(_pid *p,float IM,float VM);	//PID����Ƕ��
float PID_Calc(_pid *pid);								
float PID_Incre(_pid *pid,_pid *pidcalc);         //����ʽPID
float PID_Incre_2(_pid *pid,_pid *wheel);
float PID_POSS(_pid *pid,_pid *pidspeed,_pid *pidspeed2,int a);
int PID_POS_start(_pid *pid_position,_pid *pidspeed,float x,float y,float yaw);
void PID_Camera(_pid *pid_position,_pid *pidspeed,float x,float y,float yaw);
float PID_YAW(_pid *pid);		
float PID_YAWV(_pid *pid);
float PID_Incre3(_pid *pid);
#endif

