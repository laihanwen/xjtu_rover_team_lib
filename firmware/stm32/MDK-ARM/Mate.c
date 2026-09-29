/**
 * @file    Mate.c
 * @brief   主控制循环：统一六维动力指令 → 单次八推分配 → 去饱和 → PWM
 */

#include "Mate.h"
#include "PID.h"
#include "Move.h"
#include "imu.h"
#include "usart.h"
#include "RC.h"
#include "tim.h"
#include "iwdg.h"
#include "AuvSafety.h"
#include "AuvRcInput.h"
#include "AuvMotionTarget.h"
#include "AuvControlSource.h"
#include <math.h>

/* 现场可调系数：保留原有数值和外部可见性。 */
float yaw_xishu  = 0.00f;
float pit_xishu  = 2.00f;
float roll_xishu = 1.00f;
float rc_xishu   = 1.00f;

/* 可逆推进器中位；实际输出范围保持 1488 +/- 450。 */
float midvalue = 1488;

// YAW PID 相关
float yaw_target = 0;
float yaw_micro_rate_dps = 8.0f;          // YAW微操满杆速度，单位：度/秒
static uint8_t last_yaw_pid_state = 0;   // 上一帧YAW PID开关状态
static uint8_t last_pitch_hold_state = 0; // 上一帧SD/PITCH开关状态
/* SD=1 时的俯仰目标角。 */
static const float PIT_HOLD_DEG = 25.0f;

/* 俯仰目标斜率和主任务周期。 */
static const float PIT_SLEW_DPS = 45.0f;
static const float TASK_DT_S = 0.01f;
static const float RADIANS_TO_DEGREES = 57.29577951308232f;

// PITCH 平滑目标
static float pit_target = 0.0f;
static float pit_target_cmd = 0.0f;

/* 逻辑电机指令限值；极性在合成后统一应用。 */
static const float MOTOR_COMMAND_LIMIT = 450.0f;

static const int8_t motor_polarity[VECTOR_THRUSTER_COUNT] = {
    Motor_1Polarity, Motor_2Polarity, Motor_3Polarity, Motor_4Polarity,
    Motor_5Polarity, Motor_6Polarity, Motor_7Polarity, Motor_8Polarity
};
static volatile float last_thruster_outputs[VECTOR_THRUSTER_COUNT];

// 外部引用
extern PID_TYPE PID_pit, PID_yaw, PID_rol;
extern IWDG_HandleTypeDef hiwdg;

/**
 * @brief 按固定 T1..T8 编号将 PWM 写入实际定时器通道。
 * @note  该函数是电机编号与硬件通道映射的唯一代码入口。
 */
static void VectorThrusterPwm_Write(const float pwm[VECTOR_THRUSTER_COUNT])
{
    const float *safe_pwm = pwm;
    float neutral_pwm[VECTOR_THRUSTER_COUNT];
    uint32_t i;

    if (AuvSafety_IsArmed() == 0U) {
        for (i = 0U; i < VECTOR_THRUSTER_COUNT; ++i)
            neutral_pwm[i] = midvalue;
        safe_pwm = neutral_pwm;
    }
    for (i = 0U; i < VECTOR_THRUSTER_COUNT; ++i)
        last_thruster_outputs[i] =
            (safe_pwm[i] - midvalue) / MOTOR_COMMAND_LIMIT;
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_1, safe_pwm[0]); /* T1 */
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_4, safe_pwm[1]); /* T2 */
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_3, safe_pwm[2]); /* T3 */
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_3, safe_pwm[3]); /* T4 */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, safe_pwm[4]); /* T5 */
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_2, safe_pwm[5]); /* T6 */
    __HAL_TIM_SET_COMPARE(&htim4,  TIM_CHANNEL_3, safe_pwm[6]); /* T7 */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, safe_pwm[7]); /* T8 */
}

void Mate_GetThrusterOutputs(float output[VECTOR_THRUSTER_COUNT])
{
    uint32_t i;
    if (output == NULL) return;
    __disable_irq();
    for (i = 0U; i < VECTOR_THRUSTER_COUNT; ++i)
        output[i] = last_thruster_outputs[i];
    __enable_irq();
}

/** @brief 公共缩放整个向量，保留其在六维空间中的方向。 */
static void MotorVector_ScaleToLimit(float motor[VECTOR_THRUSTER_COUNT],
                                     float limit)
{
    float peak = 0.0f;

    for (uint32_t i = 0U; i < VECTOR_THRUSTER_COUNT; i++) {
        float magnitude = fabs(motor[i]);
        if (magnitude > peak) peak = magnitude;
    }

    if (peak > limit) {
        float scale = limit / peak;
        for (uint32_t i = 0U; i < VECTOR_THRUSTER_COUNT; i++)
            motor[i] *= scale;
    }
}

/**
 * @brief 将YAW角度或误差归一化到[-180, 180]度
 */
static float Yaw_Wrap180(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief PWM 限幅，保持原有 [midvalue-450, midvalue+450] 范围。
 */
float constrain(float a)
{
    if (a >= midvalue + MOTOR_COMMAND_LIMIT)
        return midvalue + MOTOR_COMMAND_LIMIT;
    if (a <= midvalue - MOTOR_COMMAND_LIMIT)
        return midvalue - MOTOR_COMMAND_LIMIT;
    return a;
}

/**
 * @brief  系统初始化：PID参数、IMU配置、看门狗、定时器PWM启动
 */
void Mate_Init(void)
{
    AuvControlSource_Init();
    PID_Init(&PID_yaw, 3.0f, 0.0f, 0.02f, -100, 100);
    PID_Init(&PID_pit, 5.5f, 0.0f, 0.01f, -400, 400);
    PID_Init(&PID_rol, 5.0f, 0.0f, 0.0f,  -200, 200);
    h30_configure();

    MX_IWDG_Init();

    /* 启动八推 PWM；注释与运行时 T1..T8 映射一致。 */
    HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_1);  /* T1 */
    HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_4);  /* T2 */
    HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_3);  /* T3 */
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_3);  /* T4 */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);  /* T5 */
    HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_2);  /* T6 */
    HAL_TIM_PWM_Start(&htim4,  TIM_CHANNEL_3);  /* T7 */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  /* T8 */

    /* 原 9/10 号附加推进通道仅启动并保持中位，不参与混控。 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // 启动舵机 PWM：主舵机 PA8/TIM1_CH1，SG90 PC7/TIM8_CH2
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

    /* 八推上电时统一保持中位。 */
    {
        float neutral_pwm[VECTOR_THRUSTER_COUNT];
        for (uint32_t i = 0U; i < VECTOR_THRUSTER_COUNT; i++)
            neutral_pwm[i] = midvalue;
        VectorThrusterPwm_Write(neutral_pwm);
    }
    // 主舵机初始位置
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3000);
    // SG90 上电转到 90 度：TIM8 计数频率 2 MHz，3000 计数 = 1.5 ms
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 3000);
    // 遥控数据到来前保持 SA 在中位，防止首轮任务把 SG90 拉回 0 度
    MyRCKey[SA] = 127;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, midvalue);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, midvalue);
}

/**
 * @brief  主控制任务（每10ms执行一次）
 */
void Mate_Task(void)
{
    uint8_t rc_frame[AUV_RC_FRAME_SIZE];
    uint8_t rc_fresh;
    uint8_t pi_fresh;
    AuvMotionTarget pi_target;
    AuvControlSourceDecision source;

    HAL_IWDG_Refresh(&hiwdg);

    if (imu_data_ready) imu_data_ready = 0;
    HAL_Delay(10);

    /* Copy the ISR-owned 11-byte snapshot in one short critical section. */
    __disable_irq();
    rc_fresh = AuvRcInput_CopyFreshFrame(HAL_GetTick(), rc_frame);
    pi_fresh = AuvMotionTarget_CopyFresh(HAL_GetTick(), &pi_target);
    __enable_irq();

    /*
     * Lock one command source for the complete armed interval.  A missing
     * frame from that source disarms instead of silently switching to the
     * other source, which could otherwise contain an unrelated command.
     */
    source = AuvControlSource_Update(AuvSafety_IsArmed(), pi_fresh, rc_fresh);
    if (source.disarm_required != 0U)
        (void)AuvSafety_RequestArm(0U, HAL_GetTick());
    pi_fresh = ((source.active == AUV_CONTROL_SOURCE_PI) &&
                (pi_fresh != 0U)) ? 1U : 0U;
    rc_fresh = ((source.active == AUV_CONTROL_SOURCE_RC) &&
                (rc_fresh != 0U)) ? 1U : 0U;

    if (rc_fresh != 0U) {
        RC_Translate(rc_frame);
    } else {
        /* A stale command source may never retain its last thrust command. */
        for (uint32_t i = 1U; i < MyRcLength; ++i) MyRCKey[i] = 0U;
    }

    // ===== 1. 遥控器先形成六维动力层指令，不在此处进行电机分配 =====
    VectorWrenchCommand rc_wrench;
    if (pi_fresh != 0U) {
        rc_wrench.Fx = pi_target.vx * AUV_SURGE_PWM_PER_MPS;
        rc_wrench.Fy = pi_target.vy * AUV_SWAY_PWM_PER_MPS;
        /* Depth output stays zero until a validated depth driver is integrated. */
        rc_wrench.Fz = 0.0f;
        rc_wrench.Mx = 0.0f;
        rc_wrench.My = 0.0f;
        rc_wrench.Mz = 0.0f;
    } else {
        RCWrench_Calc(&rc_wrench, MyRCKey);
    }
    if ((pi_fresh == 0U) && (rc_fresh != 0U) &&
        (AuvSafety_IsArmed() != 0U))
        RCServo_Calc(MyRCKey);

    // ===== 2. RcData[9]独立控制YAW PID，上升沿锁定当前航向 =====
    if (pi_fresh != 0U) {
        yaw_target = pi_target.yaw * RADIANS_TO_DEGREES;
        yaw_xishu = 1.0f;
        last_yaw_pid_state = 0U;
    } else if (MyRCKey[YAW_PID_SWITCH] == 1U && last_yaw_pid_state == 0U)
    {
        yaw_target = Yaw_Wrap180(Angle_Measure.yaw);
        PID_yaw.PreError = 0.0f;
        PID_yaw.Integral = 0.0f;
    }

    // PID开启后，原左右自旋摇杆用于缓慢增减YAW目标角
    if ((pi_fresh == 0U) && (MyRCKey[YAW_PID_SWITCH] == 1U))
    {
        // 符号方向与PID关闭时的直接自旋方向保持一致
        float yaw_stick = ((float)MyRCKey[2] - (float)MyRCKey[1]) / 255.0f;
        yaw_target += yaw_stick * yaw_micro_rate_dps * TASK_DT_S;
        yaw_target = Yaw_Wrap180(yaw_target);
    }
    if (pi_fresh == 0U) {
        yaw_xishu = (MyRCKey[YAW_PID_SWITCH] == 1U) ? 1.00f : 0.00f;
        last_yaw_pid_state = MyRCKey[YAW_PID_SWITCH];
    }

    // ===== 2.1 SD仍然独立控制PITCH 25度姿态 =====
    if ((pi_fresh == 0U) && (MyRCKey[SD] == 1U) &&
        (last_pitch_hold_state == 0U))
    {
        pit_target = Angle_Measure.pit;
    }
    last_pitch_hold_state = (pi_fresh != 0U) ? 0U : MyRCKey[SD];
    pit_target_cmd = ((pi_fresh == 0U) && (MyRCKey[SD] == 1U))
        ? PIT_HOLD_DEG : 0.0f;

    // ===== 2.2 让 PITCH 目标平滑变化，避免突变 =====
    {
        float max_step = PIT_SLEW_DPS * TASK_DT_S;
        float err = pit_target_cmd - pit_target;

        if (err > max_step)
            pit_target += max_step;
        else if (err < -max_step)
            pit_target -= max_step;
        else
            pit_target = pit_target_cmd;
    }

    // ===== 3. PID 计算 =====
    // YAW使用最短角度误差，避免跨越+/-180度时产生大幅突变
    float yaw_error = Yaw_Wrap180(yaw_target - Angle_Measure.yaw);
    PID_Postion_Cal(&PID_yaw, yaw_error, 0.0f);
    PID_Postion_Cal(&PID_rol, 0,          Angle_Measure.rol);
    PID_Postion_Cal(&PID_pit, pit_target, Angle_Measure.pit);

    // ===== 4. 构造唯一六维动力输入 u_dyn，再执行一次推力分配 =====
    VectorWrenchCommand dynamics_wrench;
    float motor_dev[VECTOR_THRUSTER_COUNT];

    /*
     * 平移轴来自遥控，横滚/俯仰轴来自姿态闭环；偏航轴由开关在
     * 遥控直控与航向闭环之间二选一。PID 与遥控不再各自生成八路
     * 电机量，因此不存在两套电机输出在 PWM 层直接叠加的问题。
     */
    dynamics_wrench.Fx = rc_xishu * rc_wrench.Fx;
    dynamics_wrench.Fy = rc_xishu * rc_wrench.Fy;
    dynamics_wrench.Fz = rc_xishu * rc_wrench.Fz;
    dynamics_wrench.Mx = roll_xishu * PID_rol.OutPut;
    dynamics_wrench.My = pit_xishu * PID_pit.OutPut;
    dynamics_wrench.Mz = ((pi_fresh != 0U) ||
                          (MyRCKey[YAW_PID_SWITCH] == 1U))
        ? yaw_xishu * PID_yaw.OutPut
        : rc_xishu * rc_wrench.Mz;

    /* 与论文 u_dyn -> u_thruster 顺序一致：完整六维指令只分配一次。 */
    VectorAllocate_Wrench(&dynamics_wrench, motor_dev);

    /* 单一公共比例动态缩放，保持合成六维指令的相对关系。 */
    MotorVector_ScaleToLimit(motor_dev, MOTOR_COMMAND_LIMIT);

    /* 统一应用极性，再转为 PWM。 */
    float motor_pwm[VECTOR_THRUSTER_COUNT];
    for (uint32_t i = 0U; i < VECTOR_THRUSTER_COUNT; i++) {
        motor_pwm[i] = constrain(
            midvalue + (float)motor_polarity[i] * motor_dev[i]);
    }

    // ===== 5. PWM 输出到 TIM =====
    VectorThrusterPwm_Write(motor_pwm);

    /* 原附加推进通道不参与混控，每周期强制中位。 */
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_1, midvalue);
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_2, midvalue);

    HAL_IWDG_Refresh(&hiwdg);
}
