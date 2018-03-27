#include "MPU6050.h"
#include "usart.h"
#include "delay.h"

/*******************************

为模块化的MPU6050所写，该模块自带卡尔曼滤波算法
使用前建议通过上位机对加速度、角速度做矫正
MPU_Init()函数为Z轴校准

********************************/

void MPU_Init()
{
	//角度初始化指令，0XFF,0XAA,0X01,0X04,0X00
	USART_SendData(USART2,0xFF);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
	USART_SendData(USART2,0xAA);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
	USART_SendData(USART2,0x01);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
	USART_SendData(USART2,0x04);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
	USART_SendData(USART2,0x00);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
	delay_ms(10);
}
