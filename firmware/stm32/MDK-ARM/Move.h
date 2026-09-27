/**
 * @file    Move.h
 * @brief   Unified six-axis command and eight-thruster allocation declarations.
 */

#ifndef __MOVE_H
#define __MOVE_H

#include "tim.h"
#include "PID.h"
#include "gpio.h"

#define VECTOR_THRUSTER_COUNT 8U

/*
 * Motor polarity only compensates wiring/ESC direction. The vectors documented
 * in Move.c are BODY-FORCE directions; the visible jet directions are their
 * negatives. Change a polarity only when a low-power direction test proves the
 * measured body force is opposite to the documented positive direction.
 */
#define Motor_1Polarity   1
#define Motor_2Polarity  -1
#define Motor_3Polarity  -1
#define Motor_4Polarity   1
#define Motor_5Polarity   1
#define Motor_6Polarity  -1
#define Motor_7Polarity  -1
#define Motor_8Polarity  -1

/* Paper-style unified dynamics input u_dyn=[Fx,Fy,Fz,Mx,My,Mz]. */
typedef struct {
    float Fx;
    float Fy;
    float Fz;
    float Mx;
    float My;
    float Mz;
} VectorWrenchCommand;

void RCWrench_Calc(VectorWrenchCommand *command, const uint8_t *RC);
void VectorAllocate_Wrench(
    const VectorWrenchCommand *command,
    float motor_output[VECTOR_THRUSTER_COUNT]);
void RCServo_Calc(uint8_t *RC);
int  Servo_Limit(int a);

#endif
