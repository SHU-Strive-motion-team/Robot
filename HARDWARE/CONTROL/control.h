#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "sys.h"
#include "lcd.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include <math.h>

#define PI 3.141592654f
#define BMQ_DEC 4.0f		//编码器分频
#define BMQ_MAX 500.0f		//编码器编码数
#define BMQ_T 0.01f   		//编码器计算当前速度时间
#define BMQ_L 0.2006f		//编码器轮到机器人中心的距离
#define BMQ_R 0.0275f		//编码器轮子半径
#define RAD 0.1570796327f	//编码器一个脉冲对应的角度 pi/500/4/0.01
#define MOTOR_L 0.2013f		//轮到机器人中心的距离
#define MOTOR_R 0.0508f		//轮子的半径

#define MOTOR_STATIC_1 4000		//TIM9 CH1 PE5
#define MOTOR_STATIC_2 4000  	//TIM9 CH2 PE6

#define MID_LASER 268	//激光定位中心
#define MID_VIEW 320		//视觉定位中心
#define DIS_LASER 2500			//篮筐雷达定位距离
#define DIS_VIEW 280	//篮筐视觉定位距离

struct robot{
	float X;		//机器人在坐标系中x坐标
	float Y;		//机器人在坐标系中y坐标
	float theta;	//机器人正方向和y轴夹角
	float Vx;		//机器人在坐标系x方向速度
	float Vy;		//机器人在坐标系y方向速度
	float W;		//机器人角速度，顺时针正方向
	float w[3];		//编码器的实际计数/4
	float v[3];		//编码器的速度
	float pwm[3];	//轮子的pwm
	float theta_dev;	//上一时刻，机器人theta角
	float theta_offset;	//角度偏差矫正
};

extern struct robot robot_zqd;
extern u32 uart_data[3];
extern u8 zhongquan_case;
extern u8 sanfen_case;
extern u8 changdi;
extern u8 lankuang_state;
extern u8 chengxu;
void control_init(void);
void control3_W(float W);
void control2_W(float W);
void control1_W(float W);
void shot_init(void);		//弹射初始化
void xianwei_init(void);	//限位开关初始化
void hongwai_init(void);	//红外开关初始化
void get_position(void);	//坐标转换
void charge_init(void);		//弹射充电开关初始化
//球场坐标速度转轮子的PWM
//vx：球场坐标的x轴速度
//vy：球场坐标的y轴速度
//w:机器人原地旋转的角速度
void set_motor_vx_vy_w(float vx,float vy,float w);
void set_motor_vx_vy_w_R(float vx, float vy, float w);
void jixiebi_down(void);
void jixiebi_up(void);
int down_shot_up(void);		//机械臂下降，投球，机械臂上升
int down_shot(void);
void get_hongwai(void);			//获取红外状态
void get_hongwai_dixian(float dis);
//直线行走
//X_I:目标坐标的X
//Y_I:目标坐标的Y
//V：前进的速度
//distance:电机停止的距离
void robot_straight_I(float X_I,float Y_I,float Theta,float V,float W,float distance);
void robot_straight_Y(float Y_I,float V,float distance,u32 a_start,u32 a_stop);
void robot_straight_stage(float X_I,float Y_I,float Theta_I);
void robot_straight_ObsAvoidance(float X_I,float Y_I,float Theta_I);
void robot_certain_point(float X_I,float Y_I,float Theta_I,float pointX, float pointY,float pointTheta);
//自旋运动
//W自旋角速度 Theta 目标姿态角 dis姿态角偏差
void robot_turnOrigin(float W,float Theta,float dis);
void robot_turnOrigin_stage(float theta);
void getBasketball(void);
void getVolleyball(void);

void find_ball(u8 ball);
void find_ball_laser(void);
void find_ball_zhongquan(void);
void find_lankuang(void);
void find_ball_sanfen(u8 ball);
u8 find_ball_dixian(void);
int shot(void);
void charge(u8 state);		//弹射充电开关
void go_back(void);

void remote_control(void);
u8 xianwei_down(void);
u8 xianwei_up(void);

u8 uart_getLaser(void);
u8 uart_getData(void);
void zhongquanpoint(u8 zhongquan);
void zhongquanpointfan(u8 zhongquan);
void sanfenpoint(u8 sanfen,u8 zhongquan);
void panduan_weizhi(void);
void panduan_weizhifan(void);
void panduan_weizhi2(void);
void panduan_weizhifan2(void);
void panduan_weizhisanfen(void);
void panduan_weizhisanfenfan(void);
#endif
