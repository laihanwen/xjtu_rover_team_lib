/**
 * @file    Move.c
 * @brief   Unified six-axis command and fixed eight-thruster allocation.
 */

#include "Move.h"
#include "AuvGripper.h"
#include "AuvCameraServo.h"
#include "AuvCameraServoRemote.h"
#include "RC.h"

float RCStep;
int L_Servo;

typedef enum {
    VECTOR_AXIS_FX = 0,
    VECTOR_AXIS_FY,
    VECTOR_AXIS_FZ,
    VECTOR_AXIS_MX,
    VECTOR_AXIS_MY,
    VECTOR_AXIS_MZ,
    VECTOR_AXIS_COUNT
} VectorAxis;

#define RC_COMMAND_FULL_SCALE 255.0f
#define RC_SLOW_PWM_SPAN      300.0f
#define RC_NORMAL_PWM_SPAN    450.0f

/*
 * Body frame: +X forward, +Y left, +Z up. Positions are in millimetres.
 * Order is the existing PWM order T1..T8:
 *
 * T1 ( 180,  120,  58)   T2 ( 180, -120,  58)
 * T3 ( 180, -120, -58)   T4 ( 180,  120, -58)
 * T5 (-180,  120,  58)   T6 (-180, -120,  58)
 * T7 (-180, -120, -58)   T8 (-180,  120, -58)
 *
 * Positive unit BODY-FORCE directions d (force applied to the ROV):
 * T1 ( 0.416699, -0.416699,  0.807913)
 * T2 ( 0.416699,  0.416699,  0.807913)
 * T3 ( 0.416699,  0.416699, -0.807913)
 * T4 ( 0.416699, -0.416699, -0.807913)
 * T5 ( 0.416699,  0.416699, -0.807913)
 * T6 ( 0.416699, -0.416699, -0.807913)
 * T7 ( 0.416699, -0.416699,  0.807913)
 * T8 ( 0.416699,  0.416699,  0.807913)
 *
 * Mechanical jet directions are j=-d. All positive-command jets point toward
 * -X, so pure +X uses the forward/high-thrust direction of all eight T60s.
 * Never insert jet directions directly into the allocation model.
 *
 * With p=0.416699, q=0.807913, a=0.180 m, b=0.120 m and c=0.058 m,
 * B=[d; r cross d] has rank six and mutually orthogonal rows. Per-thruster
 * moment magnitudes are 0.121118 m (roll), 0.121256 m (pitch), and 0.125010 m
 * (yaw). The retained classic-eight gains make the single-axis translation
 * capacities 2.8336T, 2.8336T and 4.0072T, approximately equal to the classic
 * values 2*sqrt(2)T, 2*sqrt(2)T and 4T.
 *
 * This per-axis-normalized Moore-Penrose allocation preserves Gao's orthogonal
 * sign topology and the classic-eight gains
 * (0.85,0.85,0.62,0.85,0.47,0.85). Columns are
 * Fx, Fy, Fz, Mx(roll), My(pitch), Mz(yaw).
 */
static const float vector_allocation_matrix[VECTOR_THRUSTER_COUNT][6] = {
    {  0.85f, -0.85f,  0.62f,  0.85f, -0.47f, -0.85f },
    {  0.85f,  0.85f,  0.62f, -0.85f, -0.47f,  0.85f },
    {  0.85f,  0.85f, -0.62f,  0.85f,  0.47f,  0.85f },
    {  0.85f, -0.85f, -0.62f, -0.85f,  0.47f, -0.85f },
    {  0.85f,  0.85f, -0.62f, -0.85f, -0.47f, -0.85f },
    {  0.85f, -0.85f, -0.62f,  0.85f, -0.47f,  0.85f },
    {  0.85f, -0.85f,  0.62f, -0.85f,  0.47f,  0.85f },
    {  0.85f,  0.85f,  0.62f,  0.85f,  0.47f, -0.85f }
};

static void VectorAllocate(const float command[VECTOR_AXIS_COUNT],
                           float motor_output[VECTOR_THRUSTER_COUNT])
{
    uint32_t motor;
    uint32_t axis;

    for (motor = 0U; motor < VECTOR_THRUSTER_COUNT; motor++) {
        float output = 0.0f;
        for (axis = 0U; axis < (uint32_t)VECTOR_AXIS_COUNT; axis++)
            output += vector_allocation_matrix[motor][axis] * command[axis];
        motor_output[motor] = output;
    }
}

void VectorAllocate_Wrench(
    const VectorWrenchCommand *wrench,
    float motor_output[VECTOR_THRUSTER_COUNT])
{
    float command[VECTOR_AXIS_COUNT];

    command[VECTOR_AXIS_FX] = wrench->Fx;
    command[VECTOR_AXIS_FY] = wrench->Fy;
    command[VECTOR_AXIS_FZ] = wrench->Fz;
    command[VECTOR_AXIS_MX] = wrench->Mx;
    command[VECTOR_AXIS_MY] = wrench->My;
    command[VECTOR_AXIS_MZ] = wrench->Mz;
    VectorAllocate(command, motor_output);
}

void RCWrench_Calc(VectorWrenchCommand *command, const uint8_t *RC)
{
    float forward_gain;
    float lateral_gain;
    float yaw_gain;

    /* Preserve the existing speed-selector behavior and per-axis mode weights. */
    if (RC[SPEED_SELECTOR] == 0U)
        RCStep = RC_SLOW_PWM_SPAN / RC_COMMAND_FULL_SCALE;
    else
        RCStep = RC_NORMAL_PWM_SPAN / RC_COMMAND_FULL_SCALE;

    if (RC[SPEED_SELECTOR] == 1U) {
        forward_gain = 1.00f;
        lateral_gain = 0.30f;
        yaw_gain = 0.20f;
    } else if (RC[SPEED_SELECTOR] == 0U) {
        forward_gain = 1.00f;
        lateral_gain = 1.00f;
        yaw_gain = 0.60f;
    } else {
        forward_gain = 0.80f;
        lateral_gain = 0.80f;
        yaw_gain = 0.60f;
    }

    command->Fx =
        (float)((int)RC[3] - (int)RC[4]) * RCStep * forward_gain;
    command->Fy =
        (float)((int)RC[8] - (int)RC[7]) * RCStep * lateral_gain;
    command->Fz =
        (float)((int)RC[5] - (int)RC[6]) * RCStep;
    command->Mx = 0.0f;
    command->My = 0.0f;
    command->Mz = (RC[YAW_PID_SWITCH] == 1U)
        ? 0.0f
        : (float)((int)RC[1] - (int)RC[2]) * RCStep * yaw_gain;
}

void RCServo_Calc(uint8_t *RC)
{
    /* SC upper is deliberately a no-output placeholder for the removed gripper. */
    AuvCameraServo_SelectRemote(RC[SC], RC[SI], RC[CAMERA_DIAL_VALID], 1U);
}

int Servo_Limit(int a)
{
    if (a >= 255) return 255;
    if (a <= 0)   return 0;
    return a;
}
