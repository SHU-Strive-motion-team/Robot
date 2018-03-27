#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/16
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
void initall_PWM(void);


void TIM14_PWM_Init(u32 arr,u32 psc);


void TIM9_ch1_PWM_Init(u32 arr,u32 psc);
void	TIM9_ch2_PWM_Init(u32 arr,u32 psc);
void 	TIM10_ch1_PWM_Init(u32 arr,u32 psc);
void 	TIM11_ch1_PWM_Init(u32 arr,u32 psc);
	
void 	TIM13_ch1_PWM_Init(u32 arr,u32 psc);
void 	TIM12_ch1_PWM_Init(u32 arr,u32 psc);
void 	TIM14_PWM_Init(u32 arr,u32 psc);	//84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz.  
void 	TIM5_ch2_PWM_Init(u32 arr,u32 psc);
void 	TIM4_ch1_PWM_Init(u32 arr,u32 psc);
void 	TIM4_ch2_PWM_Init(u32 arr,u32 psc);
void 	TIM3_ch1_PWM_Init(u32 arr,u32 psc);
void 	TIM3_ch2_PWM_Init(u32 arr,u32 psc);
void 	TIM3_ch3_PWM_Init(u32 arr,u32 psc);
void 	TIM3_ch4_PWM_Init(u32 arr,u32 psc);
	
	
void 	TIM4_ch3_PWM_Init(u32 arr,u32 psc);//I2C
void 	TIM4_ch4_PWM_Init(u32 arr,u32 psc);//I2C



#endif
