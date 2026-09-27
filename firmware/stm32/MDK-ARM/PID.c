/**
 * @file    PID.c
 * @brief   位置式 PID 控制器实现
 */

#include <stddef.h>
#include "PID.h"

// 全局 PID 实例（姿态控制）
PID_TYPE PID_pit, PID_yaw, PID_rol;

/**
 * @brief  初始化 PID 控制器参数
 */
void PID_Init(PID_TYPE *PID, float32_t P, float32_t I, float32_t D,
              float32_t min, float32_t max)
{
    if (PID == NULL || max <= min) return;

    PID->P = P;
    PID->I = I;
    PID->D = D;
    PID->OutMin = min;
    PID->OutMax = max;

    PID->Error     = 0;
    PID->PreError  = 0;
    PID->Differ    = 0;
    PID->Integral  = 0;
    PID->Ilimit    = 10.0f;
    PID->Ilimit_flag = 1;
    PID->Irang     = 100.0f;
    PID->Pout      = 0;
    PID->Iout      = 0;
    PID->Dout      = 0;
    PID->OutPut    = 0;
}

/**
 * @brief  位置式 PID 计算（角度环/速度环共用）
 * @param  target   目标值
 * @param  measure  测量值
 */
void PID_Postion_Cal(PID_TYPE *PID, float32_t target, float32_t measure)
{
    PID->Error  = target - measure;
    PID->Differ = PID->Error - PID->PreError;

    // 积分分离：误差过大时暂停积分
    if (PID->Error > PID->Ilimit)
        PID->Ilimit_flag = 0;
    else if (PID->Error < -PID->Ilimit)
        PID->Ilimit_flag = 0;
    else
        PID->Ilimit_flag = 1;

    if (PID->Ilimit_flag)
    {
        PID->Integral += PID->Error;
        // 积分限幅
        if (PID->Integral > PID->Irang)  PID->Integral = PID->Irang;
        if (PID->Integral < -PID->Irang) PID->Integral = -PID->Irang;
    }

    PID->Pout   = PID->P * PID->Error;
    PID->Iout   = PID->Ilimit_flag * PID->I * PID->Integral;
    PID->Dout   = PID->D * PID->Differ;
    PID->OutPut = PID->Pout + PID->Iout + PID->Dout;

    // 输出限幅
    if (PID->OutPut < PID->OutMin) PID->OutPut = PID->OutMin;
    if (PID->OutPut > PID->OutMax) PID->OutPut = PID->OutMax;

    PID->PreError = PID->Error;
}
