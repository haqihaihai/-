#include "BSP_All.h"

void BSP_ALL(void)
{
	USART3_Config();
	//printf("bsp_all\r\n");
	Initial_USART4(115200);
	//printf("bsp_usr4");
	UART5_DMA_INIT();
	//printf("bsp_usrdma\r\n");
	rc_init();
	//printf("bsp_usrdrc\r\n");
	pa_init();
	//printf("bsp_usrdpa\r\n");
	NVIC_Configuration();//遥控器初始化
	remote_control_init();
	remote_control_init2();
	
	//printf("bsp_usrdremote\r\n");
	//uart5_init(115200);
	//printf("bsp_usrduart5_init\r\n");
//  KEY_Init();
	//	Motor_State_Init();
//	Pos_Iocation_Init();
//	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS1_9tq,CAN_BS2_4tq,3,CAN_Mode_Normal);//CAN初始化正常模式,波特率1Mbps  3->1M 6->500kb-
//	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS1_9tq,CAN_BS2_4tq,3,CAN_Mode_Normal);
//	TIM3_Cap_Init(0XFFFF,72-1);		//以1Mhz的频率计数
	//USART6_Config();
	//USART6_IRQHandler();//  LED_Init();
}

