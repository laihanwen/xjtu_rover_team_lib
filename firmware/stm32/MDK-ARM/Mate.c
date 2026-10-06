/**
 * @file    Mate.c
 * @brief   主控制循环：统一六维动力指令 → 单次八推分配 → 去饱和 → PWM
 */

#include "Mate.h"
#include "AuvGripper.h"
#include "AuvCameraServo.h"
#include "AuvCameraServoConfig.h"
#include "PID.h"
#include "Move.h"
#include "imu.h"
#include "usart.h"
#include "RC.h"
#include "tim.h"
#include "iwdg.h"
#include "AuvSafety.h"
#include "AuvRovConfig.h"
#include "AuvLink.h"
#include "AuvThrusterDeadband.h"
#include "AuvPriorityMixer.h"
#include "AuvWaterPd.h"
#include "AuvRovDepth.h"
#include "AuvRcInput.h"
#include "AuvMotionTarget.h"
#include "AuvControlSource.h"
#include <math.h>

/* 现场可调系数：保留原有数值和外部可见性。 */
float yaw_xishu  = 0.00f;
float pit_xishu  = -2.00f;
float roll_xishu = -1.00f;
float rc_xishu   = 1.00f;

/* 用户于 2026-10-06 确认八路实测中位为 1492 us；试用限幅由配置决定。 */
float midvalue = 1492;

// YAW PID 相关
float yaw_target = 0;
float yaw_micro_rate_dps = 8.0f;          // YAW微操满杆速度，单位：度/秒
#if !AUV_ROV_MANUAL_TRIAL
static uint8_t last_yaw_pid_state = 0;   // 上一帧YAW PID开关状态
static const float TASK_DT_S = 0.01f;
static const float RADIANS_TO_DEGREES = 57.29577951308232f;
#endif
static AuvRovDepthControl depth_control;

/* 逻辑电机指令限值；极性在合成后统一应用。 */
static float MOTOR_COMMAND_LIMIT = AUV_ROV_MANUAL_TRIAL ? AUV_ROV_TRIAL_PWM_LIMIT : 450.0f;

static const int8_t motor_polarity[VECTOR_THRUSTER_COUNT] = {
    Motor_1Polarity, Motor_2Polarity, Motor_3Polarity, Motor_4Polarity,
    Motor_5Polarity, Motor_6Polarity, Motor_7Polarity, Motor_8Polarity
};
static volatile float last_thruster_outputs[VECTOR_THRUSTER_COUNT];
static uint8_t roll_correction_active, pitch_correction_active;
static AuvWaterRate pitch_rate, yaw_rate;
static uint8_t heading_locked, heading_hold_active;
static MatePidSnapshot pid_snapshot;
void Mate_GetPidSnapshot(MatePidSnapshot *snapshot)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *snapshot = pid_snapshot;
    __set_PRIMASK(mask);
}

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
    static float applied_offset[VECTOR_THRUSTER_COUNT];
    static uint32_t previous_ms;
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = (uint32_t)(now_ms-previous_ms);
    previous_ms = now_ms;
    if (elapsed_ms > 20U) elapsed_ms = 20U;

    if (AuvSafety_IsArmed() == 0U) {
        for (i = 0U; i < VECTOR_THRUSTER_COUNT; ++i)
        {
            neutral_pwm[i] = midvalue;
            applied_offset[i] = 0;
        }
        safe_pwm = neutral_pwm;
    }
#if AUV_ROV_MANUAL_TRIAL && !AUV_ROV_THRUSTER_CALIBRATION
    else {
        for (i = 0; i < VECTOR_THRUSTER_COUNT; ++i) {
            applied_offset[i] = AuvThruster_Slew(applied_offset[i], pwm[i]-midvalue,
                                               0.2f*(float)elapsed_ms);
            neutral_pwm[i] = midvalue+applied_offset[i];
        }
        safe_pwm = neutral_pwm;
    }
#endif
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
    uint32_t mask = __get_PRIMASK();
    if (output == NULL) return;
    __disable_irq();
    for (i = 0U; i < VECTOR_THRUSTER_COUNT; ++i)
        output[i] = last_thruster_outputs[i];
    __set_PRIMASK(mask);
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
    const AuvCameraServoConfig camera_servo_config = {
        AUV_CAMERA_SERVO_CALIBRATED,
        AUV_CAMERA_SERVO_MIN_CCR,
        AUV_CAMERA_SERVO_MAX_CCR,
        AUV_CAMERA_SERVO_STARTUP_CCR,
        AUV_CAMERA_SERVO_SLEW_PER_TICK
    };

    AuvControlSource_Init();
    AuvCameraServo_Init(&camera_servo_config);
    PID_Init(&PID_yaw, 3.0f, 0.0f, 0.02f, -100, 100);
#if AUV_ROV_MANUAL_TRIAL
    /* Operator-requested 10x gains; correction limits remain unchanged. */
    PID_Init(&PID_pit, 7.5f, 0.0f, 0.0f, -20, 20);
    PID_Init(&PID_rol, 7.5f, 0.0f, 0.0f, -20, 20);
#else
    PID_Init(&PID_pit, 5.5f, 0.0f, 0.01f, -400, 400);
    PID_Init(&PID_rol, 5.0f, 0.0f, 0.0f,  -200, 200);
#endif
    AuvRovDepth_Reset(&depth_control);
#if AUV_H30_CONFIGURE_ON_BOOT
    h30_configure();
#endif

    MX_IWDG_Init();

    /* 八推上电时统一保持中位。 */
    {
        float neutral_pwm[VECTOR_THRUSTER_COUNT];
        for (uint32_t i = 0U; i < VECTOR_THRUSTER_COUNT; i++)
            neutral_pwm[i] = midvalue;
        VectorThrusterPwm_Write(neutral_pwm);
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, midvalue);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, midvalue);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)AuvGripper_GetPulseUs() * 2U);
#if AUV_CAMERA_SERVO_CALIBRATED
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, AuvCameraServo_GetCcr());
#endif
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
    /* T35-L gripper signal: PA8/TIM1_CH1, 0.5 us per timer count. */
    if (AuvGripper_PwmEnabled()) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
#if AUV_CAMERA_SERVO_CALIBRATED
    /* Camera tilt servo: PC7/TIM8_CH2. Build-time calibration gate is enabled. */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
#endif

    /* Uncalibrated PWM is disabled; calibrated boot movement is explicit. */
    __HAL_TIM_SET_COMPARE(
        &htim1, TIM_CHANNEL_1, (uint32_t)AuvGripper_GetPulseUs() * 2U);
#if AUV_CAMERA_SERVO_CALIBRATED
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, AuvCameraServo_GetCcr());
#endif
    /* Remote data arrival never creates an immediate full-travel command. */
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
    AuvDepthSample depth_sample;
    uint8_t depth_fresh;
    float depth_output;
    FLOAT_Angle control_angle;
    uint32_t control_imu_ms, control_imu_sequence;
    uint8_t imu_fresh;

    HAL_IWDG_Refresh(&hiwdg);

    if (imu_data_ready) imu_data_ready = 0;
    HAL_Delay(10);

    if (AuvSafety_IsArmed() || AuvSafety_GetContext()->kill_active)
        AuvGripper_CancelBootTest();
    AuvGripper_BootTestTick(HAL_GetTick());
    AuvGripper_Tick();
    if (!AuvGripper_PwmEnabled()) HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(
        &htim1, TIM_CHANNEL_1, (uint32_t)AuvGripper_GetPulseUs() * 2U);
    AuvCameraServo_Tick(AuvSafety_IsArmed());
#if AUV_CAMERA_SERVO_CALIBRATED
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, AuvCameraServo_GetCcr());
#endif

    /* Copy the ISR-owned 11-byte snapshot in one short critical section. */
    uint32_t control_mask = __get_PRIMASK();
    __disable_irq();
    rc_fresh = AuvRcInput_CopyFreshFrame(HAL_GetTick(), rc_frame);
    pi_fresh = AuvMotionTarget_CopyFresh(HAL_GetTick(), &pi_target);
    depth_fresh = AuvDepth_CopyFresh(HAL_GetTick(), &depth_sample);
    control_angle = Angle_Measure;
    control_imu_ms=imu_last_sample_ms;
    control_imu_sequence=imu_sample_sequence;
    imu_fresh = (imu_sample_sequence != 0U &&
        (uint32_t)(HAL_GetTick() - imu_last_sample_ms) <= 250U) ? 1U : 0U;
    __set_PRIMASK(control_mask);

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

    /* Only explicit high-speed RC selection raises the manual ceiling. */
#if AUV_ROV_MANUAL_TRIAL
    MOTOR_COMMAND_LIMIT = (rc_fresh && rc_frame[6] == 2U)
        ? AUV_ROV_HIGH_PWM_LIMIT : AUV_ROV_TRIAL_PWM_LIMIT;
#endif
    // ===== 1. 遥控器先形成六维动力层指令，不在此处进行电机分配 =====
    VectorWrenchCommand rc_wrench;
    if (pi_fresh != 0U) {
        rc_wrench.Fx = pi_target.vx * AUV_SURGE_PWM_PER_MPS;
        rc_wrench.Fy = pi_target.vy * AUV_SWAY_PWM_PER_MPS;
        /* Depth control is applied after source arbitration below. */
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

    // ===== 2. RcData[8]控制YAW PID，上升沿锁定当前航向 =====
#if AUV_ROV_MANUAL_TRIAL
    heading_hold_active=AuvWater_Heading(control_angle.yaw,
        (uint8_t)(AUV_ROV_AUTO_HOLD_ENABLED && AuvSafety_IsArmed() && imu_fresh && rc_fresh),
        (uint8_t)(MyRCKey[1] || MyRCKey[2]),&heading_locked,&yaw_target);
    if(!AuvSafety_IsArmed() || !imu_fresh) {
        pitch_rate.ready=yaw_rate.ready=0;
    } else {
        AuvWater_Rate(&pitch_rate,control_angle.pit,control_imu_ms,control_imu_sequence);
        AuvWater_Rate(&yaw_rate,control_angle.yaw,control_imu_ms,control_imu_sequence);
    }
#else
    if (pi_fresh != 0U) {
        yaw_target = pi_target.yaw * RADIANS_TO_DEGREES;
        yaw_xishu = 1.0f;
        last_yaw_pid_state = 0U;
    } else if (MyRCKey[YAW_PID_SWITCH] == 1U && last_yaw_pid_state == 0U)
    {
        yaw_target = Yaw_Wrap180(control_angle.yaw);
        PID_yaw.PreError = 0.0f;
        PID_yaw.Integral = 0.0f;
    }

    // PID开启后，原左右自旋摇杆用于缓慢增减YAW目标角
    if ((pi_fresh == 0U) && (MyRCKey[YAW_PID_SWITCH] == 1U))
    {
        // 符号方向与PID关闭时的直接自旋方向保持一致
        float yaw_stick = ((float)MyRCKey[1] - (float)MyRCKey[2]) / 255.0f;
        yaw_target += yaw_stick * yaw_micro_rate_dps * TASK_DT_S;
        yaw_target = Yaw_Wrap180(yaw_target);
    }
    if (pi_fresh == 0U) {
        yaw_xishu = (MyRCKey[YAW_PID_SWITCH] == 1U) ? 1.00f : 0.00f;
        last_yaw_pid_state = MyRCKey[YAW_PID_SWITCH];
    }
#endif

    /* Manual vertical input suspends depth hold; recenter locks the new depth. */
    if (depth_control.active && !depth_fresh)
        (void)AuvSafety_RequestArm(0U, HAL_GetTick());
    depth_output = AuvRovDepth_Step(&depth_control, &depth_sample,
        (depth_fresh && AuvSafety_IsArmed() &&
         ((AUV_DEPTH_CONTROL_CALIBRATED && pi_fresh) ||
          (AUV_ROV_AUTO_HOLD_ENABLED && AUV_ROV_RELATIVE_DEPTH_ENABLED && rc_fresh &&
                      !MyRCKey[5] && !MyRCKey[6]))) ? 1U : 0U,
        pi_fresh ? 0U : 1U, pi_fresh ? pi_target.depth : 0.0f);

    // ===== 3. PID 计算 =====
    if (imu_fresh == 0U) {
        (void)AuvSafety_RequestArm(0U, HAL_GetTick());
        PID_yaw.Integral = PID_yaw.PreError = PID_yaw.OutPut = 0.0f;
        PID_rol.Integral = PID_rol.PreError = PID_rol.OutPut = 0.0f;
        PID_pit.Integral = PID_pit.PreError = PID_pit.OutPut = 0.0f;
    } else {
    // YAW使用最短角度误差，避免跨越+/-180度时产生大幅突变
    float yaw_error = Yaw_Wrap180(yaw_target - control_angle.yaw);
    PID_Postion_Cal(&PID_yaw, yaw_error, 0.0f);
    PID_Postion_Cal(&PID_rol, 0,          control_angle.rol);
    PID_Postion_Cal(&PID_pit, 0.0f, control_angle.pit);
#if AUV_ROV_MANUAL_TRIAL
    PID_pit.OutPut=AuvWater_Pd(-control_angle.pit,pitch_rate.rate,7.5f,1.5f,20.0f);
    PID_yaw.OutPut=AuvWater_Pd(yaw_error,yaw_rate.rate,10.0f,1.0f,20.0f);
#endif

    }

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
    dynamics_wrench.Fz = depth_control.active
        ? depth_output : rc_xishu * rc_wrench.Fz;
    dynamics_wrench.Mx = roll_xishu * PID_rol.OutPut;
    dynamics_wrench.My = pit_xishu * PID_pit.OutPut;
#if AUV_ROV_MANUAL_TRIAL && AUV_ROV_TRIAL_ATTITUDE_ENABLED
    if (!AuvSafety_IsArmed() || !imu_fresh) {
        roll_correction_active = pitch_correction_active = 0;
    } else {
        roll_correction_active = AuvAttitude_Active(control_angle.rol, roll_correction_active);
        pitch_correction_active = AuvAttitude_Active(control_angle.pit, pitch_correction_active);
        if(fabsf(pitch_rate.rate)>3.0f)pitch_correction_active=1;
    }
    if (!roll_correction_active) dynamics_wrench.Mx = 0;
    if (!pitch_correction_active) dynamics_wrench.My = 0;
#endif
#if AUV_ROV_MANUAL_TRIAL && !AUV_ROV_TRIAL_ATTITUDE_ENABLED
    /* Optional manual-only commissioning mode. */
    dynamics_wrench.Mx = 0.0f;
    dynamics_wrench.My = 0.0f;
#endif
    dynamics_wrench.Mz = ((pi_fresh != 0U) ||
                          (MyRCKey[YAW_PID_SWITCH] == 1U))
        ? yaw_xishu * PID_yaw.OutPut
        : rc_xishu * rc_wrench.Mz;
#if AUV_ROV_MANUAL_TRIAL
    dynamics_wrench.Mz=heading_hold_active ? AUV_ROV_YAW_FEEDBACK_SIGN*PID_yaw.OutPut : rc_xishu*rc_wrench.Mz;
    if(heading_hold_active && fabsf(Yaw_Wrap180(yaw_target-control_angle.yaw))<0.5f &&
       fabsf(yaw_rate.rate)<1.0f)dynamics_wrench.Mz=0;
#endif

    /* 与论文 u_dyn -> u_thruster 顺序一致：完整六维指令只分配一次。 */
#if AUV_ROV_MANUAL_TRIAL
    {
        VectorWrenchCommand motion=dynamics_wrench;
        VectorWrenchCommand attitude={0};
        float motion_dev[VECTOR_THRUSTER_COUNT], attitude_dev[VECTOR_THRUSTER_COUNT];
        motion.Mx=motion.My=0;
        attitude.Mx=dynamics_wrench.Mx;
        attitude.My=dynamics_wrench.My;
        /* Depth correction shares attitude priority, rather than being
         * scaled away by simultaneous surge/sway commands. */
        if(depth_control.active) {attitude.Fz=dynamics_wrench.Fz;motion.Fz=0;}
        if(heading_hold_active) {attitude.Mz=dynamics_wrench.Mz;motion.Mz=0;}
        VectorAllocate_Wrench(&motion,motion_dev);
        VectorAllocate_Wrench(&attitude,attitude_dev);
        MotorVector_ScaleToLimit(motion_dev,MOTOR_COMMAND_LIMIT);
        (void)AuvPriority_Combine(motion_dev,attitude_dev,MOTOR_COMMAND_LIMIT,motor_dev);
    }
#else
    VectorAllocate_Wrench(&dynamics_wrench, motor_dev);
    MotorVector_ScaleToLimit(motor_dev, MOTOR_COMMAND_LIMIT);
#endif

    /* 统一应用极性，再转为 PWM。 */
    /* Publish actual gated control terms, before allocation/ESC compensation. */
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        pid_snapshot.tick_ms = HAL_GetTick();
        pid_snapshot.flags = (AuvSafety_IsArmed() ? 1U : 0U) |
            (imu_fresh ? 2U : 0U) | (roll_correction_active ? 4U : 0U) |
            (pitch_correction_active ? 8U : 0U) | (heading_hold_active ? 16U : 0U) |
            (depth_control.active ? 32U : 0U) |
            ((AUV_ROV_AUTO_HOLD_ENABLED && rc_fresh) ? 64U : 0U) |
            ((AUV_ROV_AUTO_HOLD_ENABLED && AUV_ROV_RELATIVE_DEPTH_ENABLED && rc_fresh) ? 128U : 0U);
        pid_snapshot.values[0]=control_angle.rol;
        pid_snapshot.values[1]=control_angle.pit;
        pid_snapshot.values[2]=control_angle.yaw;
        pid_snapshot.values[3]=-control_angle.rol;
        pid_snapshot.values[4]=-control_angle.pit;
        pid_snapshot.values[5]=Yaw_Wrap180(yaw_target-control_angle.yaw);
        pid_snapshot.values[6]=AuvSafety_IsArmed() ? dynamics_wrench.Mx : 0;
        pid_snapshot.values[7]=AuvSafety_IsArmed() ? dynamics_wrench.My : 0;
        pid_snapshot.values[8]=(AuvSafety_IsArmed() && heading_hold_active) ? dynamics_wrench.Mz : 0;
        pid_snapshot.values[9]=pitch_rate.rate;
        pid_snapshot.values[10]=yaw_rate.rate;
        pid_snapshot.values[11]=depth_fresh ? depth_sample.depth_m : NAN;
        pid_snapshot.values[12]=depth_control.active ? depth_control.target : NAN;
        pid_snapshot.values[13]=depth_output;
        __set_PRIMASK(mask);
    }
    float motor_pwm[VECTOR_THRUSTER_COUNT];
    for (uint32_t i = 0U; i < VECTOR_THRUSTER_COUNT; i++) {
#if AUV_ROV_THRUSTER_CALIBRATION
        /* Physical PWM offset, independent of mixer polarity; all others neutral. */
        motor_pwm[i] = midvalue + AuvLink_CalibrationOffset((uint8_t)i, HAL_GetTick());
#else
#if AUV_ROV_MANUAL_TRIAL
        motor_dev[i] = AuvThruster_Compensate(motor_dev[i], MOTOR_COMMAND_LIMIT, AUV_ROV_START_OFFSET_US);
#endif
        motor_pwm[i] = constrain(
            midvalue + (float)motor_polarity[i] * motor_dev[i]);
#endif
    }

    // ===== 5. PWM 输出到 TIM =====
    VectorThrusterPwm_Write(motor_pwm);

    /* 原附加推进通道不参与混控，每周期强制中位。 */
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_1, midvalue);
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_2, midvalue);

    HAL_IWDG_Refresh(&hiwdg);
}
