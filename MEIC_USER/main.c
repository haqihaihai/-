#include "Include.h"
struct rt_timer timer1,timer2,timer3,timer4;
rt_thread_t tid = RT_NULL;
extern MOTOR_REC_T motor_receive_go8;


int main(void)
{	
//	USART6_Config();
//	rodicmd();
	//DBUS6_Init(motor_receive_go8.buf,17);
//	rt_thread_mdelay(2000);
  /* 创建线程 1 */
  tid = rt_thread_create("thread1",      //线程名字
                          thread_entry1, //线程入口函数
	                        RT_NULL,       //线程入口参数
                          1024,           //线程栈大小
                          6,             //线程优先级
							5);            //线程时间片
  if (tid != RT_NULL)
      rt_thread_startup(tid);
	
  /* 创建线程 2 */
  tid = rt_thread_create("thread2",      //线程名字
                          thread_entry2, //线程入口函数
	                        RT_NULL,       //线程入口参数
                          1024,           //线程栈大小
                          7,             //线程优先级
							5);            //线程时间片
  if (tid != RT_NULL)
      rt_thread_startup(tid);
  
  tid = rt_thread_create("thread3",      //线程名字
                          thread_entry3, //线程入口函数
	                        RT_NULL,       //线程入口参数
                          1024,           //线程栈大小
                          8,             //线程优先级
							5);            //线程时间片
  if (tid != RT_NULL)
      rt_thread_startup(tid);

//  /* 创建线程 3 */
//  tid = rt_thread_create("thread3",      //线程名字
//                          thread_entry3, //线程入口函数
//	                        RT_NULL,       //线程入口参数
//                          1024,           //线程栈大小
//                          7,             //线程优先级
//													5);            //线程时间片
//  if (tid != RT_NULL)
//      rt_thread_startup(tid);
//  /* 创建线程 4*/
//  tid = rt_thread_create("thread4",      //线程名字
//                          thread_entry4, //线程入口函数
//	                        RT_NULL,       //线程入口参数
//                          1024,           //线程栈大小
//                          6,             //线程优先级
//													5);            //线程时间片
//  if (tid != RT_NULL)
//      rt_thread_startup(tid);
//	
		rt_timer_init(&timer1, "timer1",         /* 定时器名字是timer1 */
                    timeout1,                /* 超时时回调的处理函数 */
                    RT_NULL,                 /* 超时函数的入口参数 */
					50,                     /* 定时长度,以OSTick为单位1ms*/
                    RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */
		rt_timer_start(&timer1);
//		
////		rt_timer_init(&timer2, "timer2",         /* 定时器名字是timer1 */
////                    timeout2,                /* 超时时回调的处理函数 */
////                    RT_NULL,                 /* 超时函数的入口参数 */
////                    20,                     /* 定时长度,以OSTick为单位1ms*/
////                    RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */
////    rt_timer_start(&timer2);
//		
//		rt_timer_init(&timer3, "timer3",         /* 定时器名字是timer1 */
//                    timeout3,                /* 超时时回调的处理函数 */
//                    RT_NULL,                 /* 超时函数的入口参数 */
//                    10,                     /* 定时长度,以OSTick为单位1ms*/
//                    RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */
//    rt_timer_start(&timer3);
//		
//		rt_timer_init(&timer4, "timer4",         /* 定时器名字是timer1 */
//                    timeout4,                /* 超时时回调的处理函数 */
//                    RT_NULL,                 /* 超时函数的入口参数 */
//                    20,                     /* 定时长度,以OSTick为单位1ms*/
//                    RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */
//    rt_timer_start(&timer4);

	return 0;
}
//extern _pid pid_pos[2];
//extern _pid pid_position[2];
//extern _pid pid_yaw;
//extern _pid pid_yawv;
//extern float x_pos;
//extern float y_pos;
//int iii=0;
//void aaa(int argc, char**argv)
//{
//	for(iii=0;iii<2;iii++){
//		pid_pos[iii].Kp=return_float(argv[1]);
//		pid_pos[iii].Kd=return_float(argv[2]);;
//		pid_position[iii].Kp=return_float(argv[3]);
//		pid_position[iii].Kd=return_float(argv[4]);
//	}
//	
//	x_pos=return_float(argv[5]);
//	y_pos=return_float(argv[5]);
////	pid_pos[0].Kp=return_float(argv[1]);
////	pid_pos[1].Kp=return_float(argv[1]);
////	pid_pos[0].Ki=return_float(argv[2]);
////	pid_pos[1].Ki=return_float(argv[2]);
////	pid_pos[0].Kd=return_float(argv[3]);
////	pid_pos[1].Kd=return_float(argv[3]);
////	x_pos=return_float(argv[4]);
////	y_pos=return_float(argv[4]);
//	
//	
//	//pid_pos[1].SetSpeed=return_float(argv[3]);
////	pid_yaw.Kp=return_float(argv[1]);
////	pid_yaw.Ki=return_float(argv[2]);
////	pid_yawv.Kp=return_float(argv[3]);
////	pid_yawv.Ki=return_float(argv[4]);
////	pid_yaw.SetSpeed=return_float(argv[5]);
////	printf("float:%f \r\n",pid_yaw.Kp);
////	printf("float:%f \r\n",pid_yaw.Ki);
////	printf("float:%f \r\n",pid_yawv.Kp);
////	printf("float:%f \r\n",pid_yawv.Ki);
////	printf("float:%f \r\n",pid_yaw.SetSpeed);
//	
//	
////	pid_pos[0].Kp=return_float(argv[1]);
////	pid_pos[1].Kp=return_float(argv[2]);
//////printf("int:%d",return_int(argv[1]));	
//////printf("float:%f \r\n",return_float(argv[2]));
////	printf("float:%f \r\n",pid_pos[0].Kp);
////	printf("float:%f \r\n",pid_pos[1].Kp);


////	printf("float:%f \r\n",pid_pos[0].Kp);
////	printf("float:%f \r\n",pid_pos[1].Kp);
////	printf("float:%f \r\n",pid_pos[0].Ki);
////	printf("float:%f \r\n",pid_pos[1].Ki);
////	printf("float:%f \r\n",x_pos);
////	printf("float:%f \r\n",y_pos);
//	printf("float:%f \r\n",pid_position[0].Kp);
//	printf("float:%f \r\n",pid_position[0].Ki);
//	
//	printf("xfloat:%f \r\n",x_pos);
//	printf("yfloat:%f \r\n",y_pos);
//	
//rt_thread_mdelay(500);

//}
//MSH_CMD_EXPORT(aaa, atcmd sample: atcmd <server|client>);
