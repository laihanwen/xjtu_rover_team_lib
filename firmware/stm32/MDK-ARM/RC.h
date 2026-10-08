/**
 * @file    RC.h
 * @brief   遥控器相关宏定义与函数声明
 */

#ifndef __RC_H
#define __RC_H

#include "main.h"

// 遥控器按键通道索引（对应 MyRCKey[] 数组下标）
#define SI 9                 // RcData[5]: verified SI dial, 0..255
#define SA SI                // Legacy dial alias
#define SC 11                // RcData[7]: down=0, camera=1, gripper placeholder=2
#define SB SC                // Legacy selector alias
#define SPEED_SELECTOR 10    // RcData[6]: preserve speed behavior independently
#define CAMERA_DIAL_VALID 13  // RcData[10]: explicit verified dial marker
#define SD 12                // RcData[8]: yaw hold
#define YAW_PID_SWITCH SD
#define SE YAW_PID_SWITCH
#define DEPTH_HOLD_SWITCH 14  // RcData[9]: depth hold, independent of SI

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
