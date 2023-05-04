#ifndef __INCLUDE_H
#define __INCLUDE_H	 
#include <rtthread.h>
#include "stm32f4xx.h"
#include "arm_math.h"
#include "Driver_USART.h"

#include "app_thread.h"
#include "PID.h"
#include "board.h"
#include "BSP_All.h"
#include "delay.h"   
#include "DRIVER_DBUS.h"
#include "timer.h"

#include "usart_go.h"

#include "driver_Laser_plate.h"
#include "mathematicalmadel.h"
#include "foot_trajectory.h"

#include "DMA.h"

#define start_zheng 52*588
#define start_ping1 50*588
#define start_ping2 12*588
#define kpmotor  500
#define ksmotor  100
#define radsmotor 100

#define kpmotor_li 500
#define ksmotor_li 100

#define tort_qianjin 1
#define tort_houtui 2
#define zixua 3
#define shunxuan 4
#define nixuan 5

#define dynamic_x 0.0f
#define dynamic_y 30.94f

#define leg_da	13.39f
#define leg_xi	26


#endif

