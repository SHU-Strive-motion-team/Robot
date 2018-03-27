#include "exti.h"
#include "led.h"
//#include "key.h"
#include "delay.h"
#include "usart.h"
#include "beep.h"
#include "control.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//外部中断 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   
//外部中断0服务程序
void EXTIX_Init(void)
{
 
 	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);//PC0 连接到中断线0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);//PC1 连接到中断线1
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource9);//PF9 连接到中断线9
	

    /* 配置EXTI_Line0 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE0
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = DISABLE;//使能LINE0
	  EXTI_Init(&EXTI_InitStructure);//配置
	
	  /* 配置EXTI_Line1 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE1
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = DISABLE;//使能LINE1
	  EXTI_Init(&EXTI_InitStructure);//配置
		
		/* 配置EXTI_Line9 */
		/*
	  EXTI_InitStructure.EXTI_Line = EXTI_Line9;//LINE9
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE9
	  EXTI_Init(&EXTI_InitStructure);//配置
		*/
 
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断0
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure);//配置
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//外部中断1
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure);//配置
		
		/*
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断9
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure);//配置
		*/
}


void EXTIX_Disable(u8 extix)
{
	EXTI_InitTypeDef   EXTI_InitStructure;

	if(extix == 0)
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE0
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = DISABLE;//使能LINE0
	}
	else if(extix == 1)
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE1
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = DISABLE;//使能LINE1
	}
	else if(extix==9)
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line9;//LINE9
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = DISABLE;//使能LINE9
	}
	else return;
	EXTI_Init(&EXTI_InitStructure);//配置
}

void EXTIX_Enable(u8 extix)
{
 	EXTI_InitTypeDef   EXTI_InitStructure;

	if(extix == 0)
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE0
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
	}
	else if(extix == 1)
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE1
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE1
	}
	else if(extix==9)
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line9;//LINE9
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发 
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE9
	}
	
	else return;
	EXTI_Init(&EXTI_InitStructure);//配置
}



//外部中断0服务程序 
void EXTI0_IRQHandler(void)
{
	if(xianwei_up()==1)	 	 //Up
	{				 
		TIM_SetCompare1(TIM9,MOTOR_STATIC_1);
		TIM_SetCompare2(TIM9,MOTOR_STATIC_2);	
		EXTIX_Disable(0);
	}
	EXTI_ClearITPendingBit(EXTI_Line0);			//清除中断标志位
	
}
 
//外部中断1服务程序
void EXTI1_IRQHandler(void)
{
	if(xianwei_down()==1)	  //Down
	{
		TIM_SetCompare1(TIM9,MOTOR_STATIC_1);
		TIM_SetCompare2(TIM9,MOTOR_STATIC_2);	
		EXTIX_Disable(1);
	}		 
	EXTI_ClearITPendingBit(EXTI_Line1);			//清除中断标志位
	
}

/*
void EXTI9_5_IRQHandler(void)
{
	
	TIM_SetCompare2(TIM9,MOTOR_STATIC_2); //PE6
	TIM_SetCompare1(TIM9,MOTOR_STATIC_1);	//PE5
	EXTIX_Disable(9);
	
}
*/
