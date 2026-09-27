/**
 * @file    Mate.h
 * @brief   八推全矢量主控制模块接口
 */

#ifndef __MATE_H
#define __MATE_H

#include "PID.h"
#include "Motor.h"
#include "Move.h"

void Mate_Task(void);
void Mate_Init(void);
/* 保留的 PWM 限幅接口，范围由 Mate.c 中的原有参数确定。 */
float constrain(float a);

#endif
