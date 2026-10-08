/**
 * @file    RC.c
 * @brief   11 字节遥控帧到内部方向通道的转换
 */

#include "RC.h"
#include "usart.h"
#include "gpio.h"

// 当前帧/上一帧遥控按键值，用于边缘检测
uint8_t MyRCKey[MyRcLength];
uint8_t LastMyRCKey[MyRcLength];

// 遥控器原始帧数据 (11字节)
uint8_t RcData[11];

/* 保留的兼容变量，供旧舵机控制接口使用。 */
uint8_t RC_SI_In = 0, RC_SI_Out = 0;

/**
 * @brief   原始遥控帧转换为 MyRCKey[] 内部通道。
 * @note    RcData[1..4] 中值为 127。每根摇杆被拆成两个
 *          0..255 单向通道：[1,2]=yaw，[3,4]=surge，
 *          [5,6]=heave，[7,8]=sway。RcData[5..9] 为开关/拨盘。
 */
void RC_Translate(uint8_t *RcData)
{
    for (int i = 1; i < MyRcLength; i++)
    {
        LastMyRCKey[i] = MyRCKey[i];
    }

    // 原始摇杆1：偏航 yaw -> MyRCKey[1,2]
    if (RcData[1] > 127) { MyRCKey[2] = 0; MyRCKey[1] = RC_Matching(Rc2MyRcKey(RcData[1])); }
    if (RcData[1] <= 127) { MyRCKey[1] = 0; MyRCKey[2] = RC_Matching(Rc2MyRcKey(RcData[1])); }

    // 原始摇杆2：前后 surge -> MyRCKey[3,4]
    if (RcData[2] > 127) { MyRCKey[4] = 0; MyRCKey[3] = RC_Matching(Rc2MyRcKey(RcData[2])); }
    if (RcData[2] <= 127) { MyRCKey[3] = 0; MyRCKey[4] = RC_Matching(Rc2MyRcKey(RcData[2])); }

    // 原始摇杆3：升沉 heave -> MyRCKey[5,6]
    if (RcData[3] > 127) { MyRCKey[6] = 0; MyRCKey[5] = RC_Matching(Rc2MyRcKey(RcData[3])); }
    if (RcData[3] <= 127) { MyRCKey[6] = RC_Matching(Rc2MyRcKey(RcData[3])); MyRCKey[5] = 0; }

    // 原始摇杆4：横移 sway -> MyRCKey[7,8]
    if (RcData[4] > 127) { MyRCKey[8] = 0; MyRCKey[7] = RC_Matching(Rc2MyRcKey(RcData[4])); }
    if (RcData[4] <= 127) { MyRCKey[7] = 0; MyRCKey[8] = RC_Matching(Rc2MyRcKey(RcData[4])); }

    // 按键通道
    MyRCKey[9]  = RcData[5];   // 舵机角度
    MyRCKey[10] = RcData[6];   // 独立速度档位
    MyRCKey[11] = RcData[7];   // SC: 0=不选，1=摄像头，2=主舵机占位
    MyRCKey[YAW_PID_SWITCH] = (RcData[8] != 0U) ? 1U : 0U;
    MyRCKey[DEPTH_HOLD_SWITCH] = (RcData[9] != 0U) ? 1U : 0U;

    MyRCKey[CAMERA_DIAL_VALID] = (RcData[10] == 1U) ? 1U : 0U;

    RcData[0] = 0;  // 帧头清零，防止重复解析
}

/**
 * @brief   检测SE开关是否刚刚拨入
 */
uint8_t RC_WhetherSE_IN_JustNow(void)
{
    if (MyRCKey[YAW_PID_SWITCH] == 1 && LastMyRCKey[YAW_PID_SWITCH] == 0)
        return FirstTime;
    return NotFirstTime;
}

/**
 * @brief   检测SE开关是否刚刚拨出
 */
uint8_t RC_WhetherSE_OUT_JustNow(void)
{
    if (MyRCKey[YAW_PID_SWITCH] == 0 && LastMyRCKey[YAW_PID_SWITCH] == 1)
        return FirstTime;
    return NotFirstTime;
}

/**
 * @brief   遥控器原始值(0~255)转换为标准化按键值(0~255)
 * @note    中值127对应0，左右满偏对应±255
 */
uint8_t Rc2MyRcKey(uint8_t RcNum)
{
    if (RcNum == 127) return 0;
    if (RcNum > 127)  return (uint8_t)((float)(RcNum - 127) / 128.0f * 255.0f);
    if (RcNum < 127)  return (uint8_t)((float)(127 - RcNum) / 127.0f * 255.0f);
    return 0;
}

/**
 * @brief   摇杆死区处理：小于35视为零
 */
uint8_t RC_Matching(uint8_t X)
{
    return (X <= 35) ? 0 : X;
}
