/**
 * @file    PID.h
 * @brief   PID 控制器数据结构与函数声明
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include <stdbool.h>

typedef float float32_t;

/**
 * @brief  PID 控制器结构体
 */
typedef struct {
    float32_t P, I, D;        // 比例、积分、微分系数
    float32_t Error;          // 当前误差
    float32_t PreError;       // 上一次误差
    float32_t Differ;         // 微分项 = 当前误差 - 上次误差
    float32_t Integral;       // 积分累积值
    float32_t Ilimit;         // 积分分离阈值
    float32_t Ilimit_flag;    // 积分分离标志
    float32_t Irang;         // 积分限幅范围
    float32_t Pout, Iout, Dout;  // P/I/D 分量
    float32_t OutPut;         // PID 总输出
    float32_t OutMin, OutMax; // 输出限幅
} PID_TYPE;

void PID_Postion_Cal(PID_TYPE *PID, float32_t target, float32_t measure);
void PID_Init(PID_TYPE *PID, float32_t P, float32_t I, float32_t D,
              float32_t min, float32_t max);

#endif
