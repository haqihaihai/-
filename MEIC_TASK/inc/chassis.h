#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Include.h"

typedef float fp32;
#define MOTOR_DISTANCE_TO_CENTER 0.8f
//���̵������ٶ�
#define MAX_WHEEL_SPEED 1500
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 1500
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1500
//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0

typedef struct
{
  fp32 vx;                         //�����ٶ� ǰ������ ǰΪ��
  fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  
  fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� 
  fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ��
  fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ��
  fp32 wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� 
	
	fp32 pos_x;                      //��������ϵx
	fp32 pos_y;                      //��������ϵy
	fp32 pos_z;                      //��������ϵangle
	
	fp32 px_set;                     //�趨��������ϵx
	fp32 py_set;                     //�趨��������ϵy
	fp32 pz_set;	                   //�趨������ת���ٶ�
	fp32 angle_set;                  //�趨������ת�Ƕ�
	
	int word_vx;
	int word_vy;

} chassis_move_t;

//����״̬ö�ٱ���
typedef enum Chassis_State
{
	MOVING, STOPPED,
}Chassis_State;

//·��״̬ö�ٱ���
typedef enum Path_State
{
	POT1_PATH, POT2_PATH,POT3_PATH,NULL_PATH,
}Path_State;

//������״̬
typedef struct Robot_State
{
	Chassis_State   ChassisState;
	Path_State      PathState;
	
	int FLAG_PATH;  //·����ɱ�־
	float Fixed_Angle;
}Robot_State;
extern Robot_State RobotState;
extern chassis_move_t chassis_move_control_loop;
void chassis_control_loop(void);
void Speed_Calculate(void);
void pos_speed_contral_remote(void);//�Զ���λ�ٶ�PID�ջ�
	
#endif

