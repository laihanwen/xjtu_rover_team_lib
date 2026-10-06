/**
 * @file    Mate.h
 * @brief   八推全矢量主控制模块接口
 */

#ifndef __MATE_H
#define __MATE_H

#include "PID.h"
#include "Motor.h"
#include "Move.h"

/*
 * Open-loop velocity feed-forward gains. They are deliberately compile-time
 * calibration values: replace them using measured, restrained-tank data for
 * the final hull. They never bypass the common +/-450 PWM deviation limit.
 */
#ifndef AUV_SURGE_PWM_PER_MPS
#define AUV_SURGE_PWM_PER_MPS 300.0f
#endif
#ifndef AUV_SWAY_PWM_PER_MPS
#define AUV_SWAY_PWM_PER_MPS 300.0f
#endif

void Mate_Task(void);
void Mate_Init(void);
void Mate_GetThrusterOutputs(float output[VECTOR_THRUSTER_COUNT]);
/* Last completed control iteration: angles, errors, gated PID moments, rates. */
typedef struct { uint32_t tick_ms; uint8_t flags; float values[11]; } MatePidSnapshot;
void Mate_GetPidSnapshot(MatePidSnapshot *snapshot);
/* 保留的 PWM 限幅接口，范围由 Mate.c 中的原有参数确定。 */
float constrain(float a);

#endif
