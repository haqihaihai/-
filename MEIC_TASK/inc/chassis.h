#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Include.h"

typedef float fp32;
#define MOTOR_DISTANCE_TO_CENTER 0.8f
//底盘电机最大速度
#define MAX_WHEEL_SPEED 1500
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1500
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1500
//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0

typedef struct
{
  fp32 vx;                         //底盘速度 前进方向 前为正
  fp32 vy;                         //底盘速度 左右方向 左为正  
  fp32 wz;                         //底盘旋转角速度，逆时针为正 
  fp32 vx_set;                     //底盘设定速度 前进方向 前为正
  fp32 vy_set;                     //底盘设定速度 左右方向 左为正
  fp32 wz_set;                     //底盘设定旋转角速度，逆时针为正 
	
	fp32 pos_x;                      //世界坐标系x
	fp32 pos_y;                      //世界坐标系y
	fp32 pos_z;                      //世界坐标系angle
	
	fp32 px_set;                     //设定世界坐标系x
	fp32 py_set;                     //设定世界坐标系y
	fp32 pz_set;	                   //设定世界旋转角速度
	fp32 angle_set;                  //设定世界旋转角度
	
	int word_vx;
	int word_vy;

} chassis_move_t;

//底盘状态枚举变量
typedef enum Chassis_State
{
	MOVING, STOPPED,
}Chassis_State;

//路径状态枚举变量
typedef enum Path_State
{
	POT1_PATH, POT2_PATH,POT3_PATH,NULL_PATH,
}Path_State;

//机器人状态
typedef struct Robot_State
{
	Chassis_State   ChassisState;
	Path_State      PathState;
	
	int FLAG_PATH;  //路线完成标志
	float Fixed_Angle;
}Robot_State;
extern Robot_State RobotState;
extern chassis_move_t chassis_move_control_loop;
void chassis_control_loop(void);
void Speed_Calculate(void);
void pos_speed_contral_remote(void);//自动走位速度PID闭环
	
#endif

