#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

 

void TIM3_Cap_Init(u16 arr,u16 psc);

extern u8  TIM3CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u16	TIM3CH1_CAPTURE_VAL;	//输入捕获值
#endif
