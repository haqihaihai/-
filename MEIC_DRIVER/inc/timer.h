#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

 

void TIM3_Cap_Init(u16 arr,u16 psc);

extern u8  TIM3CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u16	TIM3CH1_CAPTURE_VAL;	//���벶��ֵ
#endif
