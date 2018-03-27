#include "timer.h"
#include "usart.h"
#include "beep.h"
#include "control.h"
#include "stm32f4xx.h"
u8 time_out_flag=0;
u8 count = 0;
u8 counter=0;
float ll1[4]={0,0,0,0};
float ll2[4]={0,0,0,0};
float ll3[4]={0,0,0,0};

/****/
void TIM7_Int_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///Ê¹ÄÜTIM7Ê±ÖÓ
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//×Ô¶¯ÖØ×°ÔØÖµ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //¶¨Ê±Æ÷·ÖÆµ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //ÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//³õÊ¼»¯TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //ÔÊÐí¶¨Ê±Æ÷7¸üÐÂÖÐ¶Ï

	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //¶¨Ê±Æ÷7ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //ÇÀÕ¼ÓÅÏÈ¼¶0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //×ÓÓÅÏÈ¼¶0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM7,ENABLE); //Ê¹ÄÜ¶¨Ê±Æ÷7
}

void TIM2_Int_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///Ê¹ÄÜTIM2Ê±ÖÓ
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//×Ô¶¯ÖØ×°ÔØÖµ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //¶¨Ê±Æ÷·ÖÆµ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //ÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//³õÊ¼»¯TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //ÔÊÐí¶¨Ê±Æ÷2¸üÐÂÖÐ¶Ï
	TIM_Cmd(TIM2,ENABLE); //Ê¹ÄÜ¶¨Ê±Æ÷2
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //¶¨Ê±Æ÷2ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //ÇÀÕ¼ÓÅÏÈ¼¶0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //×ÓÓÅÏÈ¼¶0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*
Ñ¹ÉÚÍ¶Àº  ´ýÑéÖ¤
*/
void TIM7_IRQHandler(void)
{
	
			
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET)
	{
		
		if(counter<23)
		{
			counter++;
		}
		else
		{
			time_out_flag=1;
			
			if(chengxu==4||chengxu==5||chengxu==6)
			{
				if(lankuang_state==2)
					{
						control1_W(0);
						control2_W(0);
						control3_W(0);
						down_shot();						
					}
			}
			

			BEEP=1;
				
		}
		

	}
		 TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //Çå³ýÖÐ¶Ï±êÖ¾Î

	
	
}

//¶¨Ê±Æ÷2ÖÐ¶Ï·þÎñº¯Êý
void TIM2_IRQHandler(void)
{
	int16_t l1=0,l2=0,l3=0,temp=0;
	//u8 i = 0;

	
	//printf("ÖÐ¶Ï");
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //Òç³öÖÐ¶Ï
	{	
		if(count%3 == 0)
		{
			l1= hctl2020_getdata_0();
			l2= hctl2020_getdata_1();
			l3= hctl2020_getdata_2();
		}
		else if(count%3 == 1)
		{
			l2= hctl2020_getdata_1();
			l3= hctl2020_getdata_2();
			l1= hctl2020_getdata_0();
		}
		else if(count%3 == 2)
		{
			l3= hctl2020_getdata_2();
			l1= hctl2020_getdata_0();
			l2= hctl2020_getdata_1();
		}
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //Çå³ýÖÐ¶Ï±êÖ¾Î»
	
	
	
	/*
	if(abs(l1-ll1[3])>200)
	{
		l1 = ll1[3] + (ll1[0]+ll1[1]+ll1[2])/3;
	}	 
	for(i = 0; i < 2; i++)
	{
		ll1[i] = ll1[i+1];
	}
	ll1[2] = l1-ll1[3];
	ll1[3] = l1;
	
	
	if(abs(l2-ll2[3])>200)
	{
		l2 = ll2[3] + (ll2[0]+ll2[1]+ll2[2])/3;
	}	 
	for(i = 0; i < 2; i++)
	{
		ll2[i] = ll2[i+1];
	}
	ll2[2] = l2-ll2[3];
	ll2[3] = l2;
	
	
	if(abs(l3-ll3[3])>200)
	{
		l3 = ll3[3] + (ll3[0]+ll3[1]+ll3[2])/3;
	}	 
	for(i = 0; i < 2; i++)
	{
		ll3[i] = ll3[i+1];
	}
	ll3[2] = l3-ll3[3];
	ll3[3] = l3;
	*/
	
	
	robot_zqd.w[2] += l3/4;
	robot_zqd.w[1] += l2/4;
	robot_zqd.w[0] += l1/4;    
	
	/*
	//²âÊÔÓÃ£¬Í¨¹ý´®¿Ú·¢ËÍÀï³Ì¼ÆÊý¾Ý
	USART_SendData(USART1, 0x11);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
	USART_SendData(USART1, (l2>>8)&0xff);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
	USART_SendData(USART1, (l2&0xff));
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
	*/
	
	
	robot_zqd.v[0] = l1*RAD*BMQ_R;
	robot_zqd.v[1] = l2*RAD*BMQ_R;			//1.0233173f
	robot_zqd.v[2] = l3*RAD*BMQ_R;
	
	
	if(receive2)
	{
		if(USART2_RX_STA&0x8000)
		{
			temp = USART2_RX_BUF[7];
			robot_zqd.theta = ((float)((temp<<8)|USART2_RX_BUF[6]))/32768*180;
			receive2 = 0;
			USART2_RX_STA=0;
			
			robot_zqd.theta = robot_zqd.theta * PI / 180 + robot_zqd.theta_offset;
			
			while(robot_zqd.theta < 0)
				robot_zqd.theta  = robot_zqd.theta + PI + PI;
			
			while (robot_zqd.theta > 2 * PI)
				robot_zqd.theta = robot_zqd.theta - PI - PI;
		}
	}
	
	
	
	get_position();
	
	
	
	
	
	if(count++ == 0)
	{
		LCD_Show_lcj();
	}
	else if(count == 2)
	{
		LCD_Show_v();
	}
	else if(count == 3)
	{
		LCD_Show_V();
	}
	else if(count == 4)
	{
		LCD_Show_position();
	}
	
		
	
}



