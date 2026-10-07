/**
 * @file    RC.h
 * @brief   遥控器相关宏定义与函数声明
 */

#ifndef __RC_H
#define __RC_H

#include "main.h"

// 遥控器按键通道索引（对应 MyRCKey[] 数组下标）
#define SA  9   // 拨盘开关（对应RcData[5]，控制舵机目标角度值）
#define SB  11  // 舵机通道选择（对应RcData[7]，选择TIM通道输出舵机PWM）
#define SC  10  // 速度档位选择（对应RcData[6]，0=慢, 1=中, 2=快）
#define SD 12 // RcData[8]/btn0: yaw hold
#define YAW_PID_SWITCH SD
#define SE YAW_PID_SWITCH
#define SI 14
#define DEPTH_HOLD_SWITCH SI // RcData[9]/btn1: depth hold

// 帧头（0xA5用于串口帧同步）
#define RcKey  0xA5

// 边缘检测标志
#define FirstTime   1
#define NotFirstTime 0

// MyRCKey 使用下标 1..14，下标 0 保留。
#define MyRcLength 15

// 遥控器断连判定阈值（相邻值与中值的最大偏差）
#define StopValue  5

// 全局变量声明
extern uint8_t MyRCKey[];
extern uint8_t LastMyRCKey[];
extern uint8_t RC_SI_In, RC_SI_Out;

// 函数声明
void   RC_Translate(uint8_t *RcData);
uint8_t Rc2MyRcKey(uint8_t RcNum);
uint8_t RC_Matching(uint8_t X);
uint8_t RC_WhetherSE_IN_JustNow(void);
uint8_t RC_WhetherSE_OUT_JustNow(void);
void   HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif
