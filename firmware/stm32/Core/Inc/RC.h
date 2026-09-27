#ifndef __RC_H
#define __RC_H

#include "main.h"
//这里由于遥杆自由度不够了，所以ROLL方向的左旋和右旋没有对应到遥控器上，也就是说遥控器无法控制ROLL
//typedef struct
//{
//	uint8_t Key1;//前进
//	uint8_t Key2;//后退
//	uint8_t Key3;//横向左移
//	uint8_t Key4;//横向右移
//	uint8_t Key5;//上
//	uint8_t Key6;//下
//	uint8_t Key7;//Yaw左转
//	uint8_t Key8;//Yaw右转

//	uint8_t Key9;//SA//整体动力开关
//	uint8_t Key10;//SB//控制方向选项

//  uint8_t Key11;//SC//控制舵机选项
//  uint8_t Key12;//SD//PID选项
//  uint8_t Key13;//SE//舵机运动按键
//  uint8_t Key14;//SI//舵机位置调控
//}RC_Ctl;//都是正值，后面都是按照正值算的，比如前进时Key1不为0，但后退Key2一定为0

#define SA 9
#define SB 10
#define SC 11
#define SD 12
#define SE 13
#define SI 14

#define RcMulti 11  //这里算上了0xA5(上下，左右转，前后， 左右平移，舵机，重启， 开机保护，帧头）
#define RcKey    0xA5

#define FirstTime  1
#define NotFirstTime 0
#define Stop  1
#define Running  0

#define MyRcLength 15 //这个有算上0xA5(第一个)
#define StopValue  5 //发现遥控器由于接触的问题不容易达到0，因此设定一个范围

extern uint8_t MyRCKey[];
extern uint8_t LastMyRCKey[];
extern uint8_t RC_SI_In, RC_SI_Out;

void RC_Translate(uint8_t *RcData);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*huart,uint16_t Size);
	
uint8_t RC_WhetherYawStopJustNow(void);
uint8_t RC_WhetherYawStop(void);
uint8_t RC_WhetherRollStop(void);
uint8_t RC_WhetherSE_IN_JustNow(void);
uint8_t RC_WhetherSE_OUT_JustNow(void);

uint8_t Rc2MyRcKey(uint8_t RcNum);
uint8_t RC_Matching_F(uint8_t X);
uint8_t RC_Matching(uint8_t X);
uint8_t RC_Matching_Spin(uint8_t X);
#endif
