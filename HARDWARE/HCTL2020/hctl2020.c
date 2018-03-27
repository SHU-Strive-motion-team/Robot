#include "HCTL2020.h"
#include "delay.h" 
#include "usart.h"
/*!
 *  @brief      hctl2020 接口初始化
 *  @since      v5.0
 *  Sample usage:            hctl2020_init();   //初始化 hctl2020
 */
 
 #define time 1
 
void hctl2020_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOA,GPIOE时钟
  //F0~7解码器数据读入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; //KEY0 KEY1 KEY2对应引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
	GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12; //10-1,11-2,12,3 解码器0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC
	
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB,RES1时钟
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//|GPIO_Pin_12; //10-1,11-2,12,3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOB,RES1时钟
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; //10-1,11-2,12,3 SEL1,OE1,  SEL2,RES2,OE2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIOG
}
/*!
 *  @brief      获取脉冲数
 *  @since      v5.0
 *  Sample usage:            hctl2020_getdata_l();   hctl2020_getdata_l
*/
uint8_t HCTL2020_DATA()
{
	uint8_t l,m;


			m=PFin(7);
			l=m;
			l<<=1;
			m=PFin(6);
			l=l|m;
			l<<=1;
				m=PFin(5);
			l=l|m;
			l<<=1;
				m=PFin(4);
			l=l|m;
			l<<=1;
				m=PFin(3);
			l=l|m;
			l<<=1;
				m=PFin(2);
			l=l|m;
			l<<=1;
						m=PFin(1);
			l=l|m;
			l<<=1;
						m=PFin(0);
			l=l|m;
			//printf("HCTL2020_DATA:%d  us\r\n",l);
	return l;
	
}




int16_t hctl2020_getdata_0()
{
	
	uint8_t l=0,h=0;
	//  delay_us(time);
	HCTL2020_SEL0=0;
	HCTL2020_OE0=0;
	delay_us(20);
	h=HCTL2020_DATA();
	HCTL2020_SEL0=1;
	HCTL2020_OE0=0;
	delay_us(time);
	l=HCTL2020_DATA();
	HCTL2020_OE0=1;
	HCTL2020_RST0=0;
	delay_us(time);
	HCTL2020_RST0=1;

	
	
	HCTL2020_OE0=1;
	HCTL2020_RST0=0;
	delay_us(time);
	HCTL2020_RST0=1;
	HCTL2020_OE0=1;
	HCTL2020_RST0=0;
	delay_us(time);
	HCTL2020_RST0=1;
	
	
	
	
	return (h<<8)+l; 
	
}

   int16_t hctl2020_getdata_1()
{

	uint8_t l=0,h=0;
	//  delay_us(time);
	HCTL2020_SEL1=0;
  HCTL2020_OE1=0;
  delay_us(20);
  h=HCTL2020_DATA();

	//printf("one:%d  us\r\n",l);
  HCTL2020_SEL1=1;
	HCTL2020_OE1=0;
  delay_us(time);
  l=HCTL2020_DATA();
	//printf("one:       %d  us\r\n",h);
  HCTL2020_OE1=1;
  HCTL2020_RST1=0;
  delay_us(time);
  HCTL2020_RST1=1;
	
	
		  HCTL2020_OE1=1;
  HCTL2020_RST1=0;
  delay_us(time);
  HCTL2020_RST1=1;
	  HCTL2020_OE1=1;
  HCTL2020_RST1=0;
  delay_us(time);
  HCTL2020_RST1=1;
	
	
	
  return (h<<8)+l; 
}


   int16_t hctl2020_getdata_2()
{

	
	uint8_t l=0,h=0;
	//  delay_us(time);
	HCTL2020_SEL2=0;
  HCTL2020_OE2=0;
  delay_us(20);
  h=HCTL2020_DATA();
	//printf("one:%d  us\r\n",h);
  HCTL2020_SEL2=1;
	HCTL2020_OE2=0;
  delay_us(time);
  l=HCTL2020_DATA();
	//printf("one:       %d  us\r\n",l);
  HCTL2020_OE2=1;
  HCTL2020_RST2=0;
  delay_us(time);
  HCTL2020_RST2=1;
	
	
		  HCTL2020_OE2=1;
  HCTL2020_RST2=0;
  delay_us(time);
  HCTL2020_RST2=1;
	  HCTL2020_OE2=1;
  HCTL2020_RST2=0;
  delay_us(time);
  HCTL2020_RST2=1;
	
	
	
  return (h<<8)+l; 
	
	
}
void hctl2020_RST0(void)
{    HCTL2020_RST0=0;
  delay_us(time);
  HCTL2020_RST0=1;
}
void hctl2020_RST1(void)
{  HCTL2020_RST1=0;
  delay_us(time);
  HCTL2020_RST1=1;
}
void hctl2020_RST2(void)
{  HCTL2020_RST2=0;
  delay_us(time);
  HCTL2020_RST2=1;
}
 

/*****************************修改后代码如下***************************************************************/
/*
uint8_t HCTL2020_DATA()
{
	uint8_t l=0,m;
  uint16_t hcl=0;
	hcl=GPIO_ReadInputData(GPIOF);
	l|=hcl&0xff;

//			m=PFin(7);
//			l=m;
//			l<<=1;
//			m=PFin(6);
//			l=l|m;
//			l<<=1;
//			m=PFin(5);
//			l=l|m;
//			l<<=1;
//			m=PFin(4);
//			l=l|m;
//			l<<=1;
//			m=PFin(3);
//			l=l|m;
//			l<<=1;
//			m=PFin(2);
//			l=l|m;
//			l<<=1;
//			m=PFin(1);
//			l=l|m;
//			l<<=1;
//			m=PFin(0);
//			l=l|m;
			
	return l;
	
}




int16_t hctl2020_getdata_0()
{
	
	uint8_t l=0,h=0;
	
	HCTL2020_OE0=0;
	HCTL2020_SEL0=0;
	delay_us(1);
	
	h=HCTL2020_DATA();
	
	HCTL2020_SEL0=1;
	HCTL2020_OE0=0;
	delay_us(time);
	
	l=HCTL2020_DATA();
	
	//HCTL2020_OE0=1;
	HCTL2020_RST0=0;
	delay_us(time);
	
	HCTL2020_RST0=1;
	HCTL2020_OE0=1;
//	HCTL2020_RST0=0;
//	delay_us(time);
	
//	HCTL2020_RST0=1;
//	HCTL2020_OE0=1;
//	HCTL2020_RST0=0;
//	delay_us(time);
//	HCTL2020_RST0=1;

	return (h<<8)+l; 
	
}

   int16_t hctl2020_getdata_1()
{

	uint8_t l=0,h=0;
	HCTL2020_SEL1=0;
  HCTL2020_OE1=0;
  delay_us(time);
  h=HCTL2020_DATA();
	
  HCTL2020_SEL1=1;
	HCTL2020_OE1=0;
  delay_us(time);
  l=HCTL2020_DATA();
	
//  HCTL2020_OE1=1;
	
	
  HCTL2020_RST1=0;
  delay_us(time);
  HCTL2020_RST1=1;	
  HCTL2020_OE1=1;
	
	
//  HCTL2020_RST1=0;
//  delay_us(time);
//  HCTL2020_RST1=1;
//	HCTL2020_OE1=1;
//  HCTL2020_RST1=0;
//  delay_us(time);
//  HCTL2020_RST1=1;
	
	
	
  return (h<<8)+l; 
}


int16_t hctl2020_getdata_2()
{
	uint8_t l=0,h=0;
	
	HCTL2020_SEL2=0;
  HCTL2020_OE2=0;
  delay_us(time);
	
  h=HCTL2020_DATA();
	
  HCTL2020_SEL2=1;
	HCTL2020_OE2=0;
  delay_us(time);
	
  l=HCTL2020_DATA();
	
//  HCTL2020_OE2=1;
	
  HCTL2020_RST2=0;
  delay_us(time);
  HCTL2020_RST2=1;
	HCTL2020_OE2=1;
	
//  HCTL2020_RST2=0;
//  delay_us(time);
//  HCTL2020_RST2=1;
//	HCTL2020_OE2=1;
//  HCTL2020_RST2=0;
//  delay_us(time);
//  HCTL2020_RST2=1;
//	
	
	
  return (h<<8)+l; 
	
	
}
//void hctl2020_RST0(void)
//{    HCTL2020_RST0=0;
//  delay_us(time);
//  HCTL2020_RST0=1;
//}
//void hctl2020_RST1(void)
//{  HCTL2020_RST1=0;
//  delay_us(time);
//  HCTL2020_RST1=1;
//}
//void hctl2020_RST2(void)
//{  HCTL2020_RST2=0;
//  delay_us(time);
//  HCTL2020_RST2=1;
//}
*/

