#include "chassis.h"
#include "arm_math.h"
int16_t elecurrent[4];
chassis_move_t chassis_move_control_loop;
Robot_State RobotState;
void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    wheel_speed[0] =  vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] =  vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] =  vx_set - vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] =  vx_set + vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set;
}
void Speed_Calculate(void)
{
		//将世界坐标转换为机器坐标
		double COS, SIN;	
	
		COS = arm_cos_f32(chassis_move_control_loop.pos_z * PI / 180);
		SIN = arm_sin_f32(chassis_move_control_loop.pos_z * PI / 180);		
	
		chassis_move_control_loop.vx_set = ( chassis_move_control_loop.word_vx * COS + chassis_move_control_loop.word_vy * SIN);
		chassis_move_control_loop.vy_set = ( -chassis_move_control_loop.word_vx * SIN + chassis_move_control_loop.word_vy * COS);
		chassis_move_control_loop.wz_set = PID_Calc(&stPID_Angle,17,0.01,10,chassis_move_control_loop.angle_set,chassis_move_control_loop.pos_z);
}
void pos_speed_contral_remote()//自动走位速度PID闭环
{
	chassis_move_control_loop.word_vx=PID_Calc(&stPID_SX,3.5,0.01,0,chassis_move_control_loop.px_set,chassis_move_control_loop.pos_x);
  chassis_move_control_loop.word_vy=PID_Calc(&stPID_SY,3.5,0.01,0,chassis_move_control_loop.py_set,chassis_move_control_loop.pos_y);
	if(fabs(chassis_move_control_loop.px_set-chassis_move_control_loop.pos_x)<5)chassis_move_control_loop.word_vx=0;
	if(fabs(chassis_move_control_loop.py_set-chassis_move_control_loop.pos_y)<5)chassis_move_control_loop.word_vy=0;
}

void chassis_control_loop()
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
		
		Speed_Calculate();//将世界坐标转换为机器坐标
		
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop.vx_set,
                                          chassis_move_control_loop.vy_set, 
		                                      chassis_move_control_loop.wz_set, wheel_speed);

    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        Motor[i].ExpectSpeed = wheel_speed[i];
        temp = fabs(Motor[i].ExpectSpeed);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            Motor[i].ExpectSpeed *= vector_rate;
        }
    }

    //计算pid//赋值电流值

    elecurrent[0]=PID_Calc(&Motor[0].pid_speed,5,2.3,0,Motor[0].ExpectSpeed,Motor[0].NowSpeed);
    elecurrent[1]=PID_Calc(&Motor[1].pid_speed,5,2.3,0,Motor[1].ExpectSpeed,Motor[1].NowSpeed);
    elecurrent[2]=PID_Calc(&Motor[2].pid_speed,3,2.3,0,Motor[2].ExpectSpeed,Motor[2].NowSpeed);
    elecurrent[3]=PID_Calc(&Motor[3].pid_speed,3,2.3,0,Motor[3].ExpectSpeed,Motor[3].NowSpeed);

		Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);
}


