/**
 * @file AuvLink.c
 * @brief Raspberry Pi UART link and safety integration.
 */

#include "AuvLink.h"

#include "AuvProtocol.h"
#include "AuvGripper.h"
#include "AuvGripperConfig.h"
#include "AuvCameraServo.h"
#include "AuvDepth.h"
#include "AuvControlSource.h"
#include "AuvSafety.h"
#include "AuvMotionTarget.h"
#include "imu.h"
#include "Mate.h"
#include "usart.h"
#include "AuvRovConfig.h"
#include "AuvRcInput.h"

#ifndef AUV_LINK_UART_HANDLE
#if AUV_ROV_MANUAL_TRIAL
#define AUV_LINK_UART_HANDLE huart2
#else
#define AUV_LINK_UART_HANDLE huart3
#endif
#endif

#define AUV_LINK_BAUD_RATE          115200U
#define AUV_STATUS_PERIOD_MS        100U
#define AUV_IMU_PERIOD_MS           20U
#define AUV_DEPTH_PERIOD_MS         100U
#define AUV_ACTUATOR_PERIOD_MS      100U
#define AUV_IMU_TIMEOUT_MS          250U
#define AUV_DEGREES_TO_RADIANS      0.01745329251994329577f
#define AUV_QUIET_NAN_BITS          0x7FC00000UL
#define AUV_ERROR_HEARTBEAT_TIMEOUT (1UL << 0)
#define AUV_ERROR_SENSOR_INVALID    (1UL << 1)
#define AUV_ERROR_KILL              (1UL << 3)

static AuvProtocolParser parser;
static uint8_t rx_byte;
static volatile uint8_t ack_pending;
static volatile uint8_t ack_type;
static volatile uint8_t ack_result;
static volatile uint32_t ack_sequence;
static volatile uint8_t level_ack_pending, level_ack_result;
static volatile uint32_t level_ack_sequence;
static uint32_t status_sequence;
static uint32_t level_sequence;
static uint8_t level_sequence_seen;
static uint32_t last_status_ms;
static uint32_t last_depth_ms;
static uint32_t last_imu_ms;
static uint32_t last_actuator_ms;
static uint32_t last_imu_sequence;
static uint32_t depth_sequence;
static uint32_t actuator_sequence;

static uint8_t external_safety_seen;
static volatile uint32_t trial_arm_start_ms;
static volatile uint32_t calibration_start_ms;
static volatile uint32_t calibration_sequence;
static volatile uint8_t calibration_motor;
static volatile int16_t calibration_offset;

int16_t AuvLink_CalibrationOffset(uint8_t motor, uint32_t now_ms)
{
    if (!AuvSafety_IsArmed()) { calibration_offset = 0; return 0; }
    if ((uint32_t)(now_ms - calibration_start_ms) >= 1000U) calibration_offset = 0;
    return motor == calibration_motor ? calibration_offset : 0;
}
static uint8_t external_kill_active;
static uint8_t external_sensors_valid;

static UART_HandleTypeDef *LinkUart(void)
{
    return &AUV_LINK_UART_HANDLE;
}

static void QueueAck(uint8_t type, uint8_t result, uint32_t sequence)
{
    /* Repeated DISARM ACKs must not overwrite a calibration response. */
    if (type == AUV_MSG_CALIBRATE_LEVEL) {
        level_ack_result = result;
        level_ack_sequence = sequence;
        level_ack_pending = 1U;
        return;
    }
    ack_type = type;
    ack_result = result;
    ack_sequence = sequence;
    ack_pending = 1U;
}

static void DispatchFrame(const AuvProtocolFrame *frame, uint32_t now_ms)
{
    if ((frame->message_type == AUV_MSG_HEARTBEAT) &&
        (frame->payload_length == 8U)) {
        AuvSafety_OnHeartbeat(now_ms);
    } else if (frame->message_type == AUV_MSG_SET_ARMED) {
        uint32_t sequence = 0U;
        AuvArmResult result = AUV_ARM_MALFORMED;
        if (frame->payload_length == 5U) {
            sequence = AuvProtocol_ReadU32Le(frame->payload);
#if AUV_ROV_MANUAL_TRIAL
            if (frame->payload[4] == 1U &&
                (!AuvRcInput_CanArm(now_ms) || !imu_level_calibrated))
                result = AUV_ARM_UNSAFE;
            else
#endif
            result = AuvSafety_RequestArm(frame->payload[4], now_ms);
#if AUV_ROV_MANUAL_TRIAL
            if (result == AUV_ARM_ACCEPTED && frame->payload[4] == 1U)
                trial_arm_start_ms = now_ms;
#endif
        }
        QueueAck(AUV_MSG_SET_ARMED, (uint8_t)result, sequence);
    } else if (frame->message_type == AUV_MSG_CALIBRATE_LEVEL) {
        uint32_t seq = frame->payload_length >= 4U ? AuvProtocol_ReadU32Le(frame->payload) : 0U;
        uint8_t result = (uint8_t)AUV_ARM_UNSAFE;
        if (frame->payload_length == 5U && frame->payload[4] == 1U &&
            (!level_sequence_seen || (int32_t)(seq-level_sequence) > 0) &&
            !AuvSafety_IsArmed()) {
            level_sequence = seq;
            level_sequence_seen = 1U;
            if (imu_calibrate_level()) result = (uint8_t)AUV_ARM_ACCEPTED;
        }
        QueueAck(AUV_MSG_CALIBRATE_LEVEL, result, seq);
    } else if (frame->message_type == AUV_MSG_RC_TARGET) {
#if AUV_ROV_MANUAL_TRIAL
        if (frame->payload_length == 16U) {
            uint8_t ok = AuvRcInput_AcceptCrc(AuvProtocol_ReadU32Le(frame->payload),
                frame->payload + 4, frame->payload[15], now_ms);
            if (!ok || !frame->payload[15]) {
                external_kill_active = 1U;
                (void)AuvSafety_RequestArm(0U, now_ms);
            }
        }
#endif
    } else if (frame->message_type == 0x07U) {
#if AUV_ROV_THRUSTER_CALIBRATION
        if (frame->payload_length == 7U) {
            uint32_t seq = AuvProtocol_ReadU32Le(frame->payload);
            int16_t offset = (int16_t)((uint16_t)frame->payload[5] | ((uint16_t)frame->payload[6] << 8));
            if ((int32_t)(seq - calibration_sequence) > 0 &&
                AuvSafety_IsArmed() && AuvRcInput_CanArm(now_ms) &&
                frame->payload[4] < 8U && offset >= -75 && offset <= 75 &&
                calibration_offset == 0 && (uint32_t)(now_ms-calibration_start_ms) >= 2000U) {
                calibration_sequence = seq;
                calibration_motor = frame->payload[4];
                calibration_start_ms = now_ms;
                calibration_offset = offset;
            }
        }
#endif
    } else if (frame->message_type == AUV_MSG_REMOTE_KILL) {
#if AUV_ROV_MANUAL_TRIAL
        if (frame->payload_length == 1U && frame->payload[0] <= 1U) {
            if (frame->payload[0] || AuvRcInput_CanArm(now_ms)) {
                external_kill_active = frame->payload[0];
                if (external_kill_active) (void)AuvSafety_RequestArm(0U,now_ms);
            }
        }
#endif
    } else if (frame->message_type == AUV_MSG_MOTION_TARGET) {
        uint32_t sequence = (frame->payload_length >= 4U)
            ? AuvProtocol_ReadU32Le(frame->payload) : 0U;
#if AUV_ROV_MANUAL_TRIAL
        AuvArmResult result = AUV_ARM_UNSUPPORTED;
#else
        AuvArmResult result = AuvMotionTarget_Accept(
            frame->payload, frame->payload_length, now_ms, AuvSafety_IsArmed());
#endif
        QueueAck(AUV_MSG_MOTION_TARGET, (uint8_t)result, sequence);
    } else if (frame->message_type == AUV_MSG_ACTUATOR_COMMAND) {
        uint32_t sequence = (frame->payload_length >= 4U)
            ? AuvProtocol_ReadU32Le(frame->payload) : 0U;
        AuvArmResult result = AUV_ARM_MALFORMED;
        const uint8_t pi_authorized =
            (AuvControlSource_GetActive() == AUV_CONTROL_SOURCE_PI) ? 1U : 0U;
        if (frame->payload_length == 9U) {
            if (frame->payload[4] == AUV_GRIPPER_ACTUATOR_ID)
                result = AuvGripper_Accept(frame->payload, frame->payload_length,
                                           AuvSafety_IsArmed(), pi_authorized);
            else if (frame->payload[4] == AUV_CAMERA_SERVO_ACTUATOR_ID)
                result = AuvCameraServo_Accept(frame->payload,
                                               frame->payload_length,
                                               AuvSafety_IsArmed(),
                                               pi_authorized);
            else
                result = AUV_ARM_UNSUPPORTED;
        }
        QueueAck(frame->message_type, (uint8_t)result, sequence);
    }
}

static void SendFrame(uint8_t type, const uint8_t *payload, uint8_t length)
{
    uint8_t encoded[AUV_PROTOCOL_MAX_FRAME_SIZE];
    size_t encoded_size = AuvProtocol_Encode(
        type, payload, length, encoded, sizeof(encoded));
    if (encoded_size != 0U)
        (void)HAL_UART_Transmit(LinkUart(), encoded, (uint16_t)encoded_size, 10U);
}

static void SendPendingAck(void)
{
    uint8_t payload[6];
    uint32_t sequence;
    uint8_t type;
    uint8_t result;

    if (ack_pending == 0U && level_ack_pending == 0U) return;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (level_ack_pending) {
        type = AUV_MSG_CALIBRATE_LEVEL;
        result = level_ack_result;
        sequence = level_ack_sequence;
        level_ack_pending = 0U;
    } else {
        type = ack_type;
        result = ack_result;
        sequence = ack_sequence;
        ack_pending = 0U;
    }
    __set_PRIMASK(mask);
    payload[0] = type;
    payload[1] = result;
    AuvProtocol_WriteU32Le(&payload[2], sequence);
    SendFrame(AUV_MSG_ACK, payload, sizeof(payload));
}

static void SendStatus(uint32_t now_ms)
{
    AuvSafetyContext safety;
    const AuvSafetyContext *context = &safety;
    uint8_t payload[46] = {0};
    float thruster_outputs[VECTOR_THRUSTER_COUNT];
    uint8_t state_flags = 0U;
    uint32_t error_flags = 0U;
    AuvDepthSample depth;
    FLOAT_Angle angle;
    uint8_t depth_fresh, imu_fresh;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Heartbeat, sample and evaluation time belong to one snapshot. */
    now_ms = HAL_GetTick();
    safety = *AuvSafety_GetContext();
    depth_fresh = AuvDepth_CopyFresh(now_ms, &depth);
    angle = Angle_Measure;
    imu_fresh = (imu_sample_sequence != 0U) &&
        ((uint32_t)(now_ms - imu_last_sample_ms) <= AUV_IMU_TIMEOUT_MS);
    if (imu_level_calibrated) state_flags |= 1U << 3;
    __set_PRIMASK(mask);

    if (context->state == AUV_SAFETY_ARMED) state_flags |= 1U << 0;
    if (context->state == AUV_SAFETY_FAILSAFE) state_flags |= 1U << 2;
    if ((context->heartbeat_seen != 0U) &&
        ((uint32_t)(now_ms - context->last_heartbeat_ms) >
         AUV_HEARTBEAT_TIMEOUT_MS)) error_flags |= AUV_ERROR_HEARTBEAT_TIMEOUT;
    if (context->sensors_valid == 0U) error_flags |= AUV_ERROR_SENSOR_INVALID;
    if (context->kill_active != 0U) error_flags |= AUV_ERROR_KILL;

    AuvProtocol_WriteU32Le(&payload[0], status_sequence++);
    payload[4] = state_flags;
    AuvProtocol_WriteU32Le(&payload[5], error_flags);
    AuvProtocol_WriteU32Le(&payload[9], AUV_QUIET_NAN_BITS);  /* voltage unavailable */
    if (depth_fresh != 0U)
        AuvProtocol_WriteF32Le(&payload[13], depth.depth_m);
    else
        AuvProtocol_WriteU32Le(&payload[13], AUV_QUIET_NAN_BITS);
    if (imu_fresh != 0U) {
        AuvProtocol_WriteF32Le(&payload[17], angle.rol * AUV_DEGREES_TO_RADIANS);
        AuvProtocol_WriteF32Le(&payload[21], angle.pit * AUV_DEGREES_TO_RADIANS);
        AuvProtocol_WriteF32Le(&payload[25], angle.yaw * AUV_DEGREES_TO_RADIANS);
    } else {
        AuvProtocol_WriteU32Le(&payload[17], AUV_QUIET_NAN_BITS);
        AuvProtocol_WriteU32Le(&payload[21], AUV_QUIET_NAN_BITS);
        AuvProtocol_WriteU32Le(&payload[25], AUV_QUIET_NAN_BITS);
    }
    Mate_GetThrusterOutputs(thruster_outputs);
    payload[29] = VECTOR_THRUSTER_COUNT;
    for (uint32_t i = 0U; i < VECTOR_THRUSTER_COUNT; ++i) {
        float normalized = thruster_outputs[i];
        int16_t encoded;
        if (normalized > 1.0f) normalized = 1.0f;
        if (normalized < -1.0f) normalized = -1.0f;
        encoded = (int16_t)(normalized * 1000.0f);
        AuvProtocol_WriteI16Le(&payload[30U + 2U * i], encoded);
    }
    SendFrame(AUV_MSG_STATUS, payload, sizeof(payload));
}

static void SendPidDiagnostic(void)
{
    uint8_t payload[49];
    MatePidSnapshot snapshot;
    Mate_GetPidSnapshot(&snapshot);
    AuvProtocol_WriteU32Le(payload, snapshot.tick_ms);
    payload[4] = snapshot.flags;
    for (uint32_t i=0; i<11; ++i)
        AuvProtocol_WriteF32Le(&payload[5+4*i], snapshot.values[i]);
    SendFrame(AUV_MSG_PID_DIAGNOSTIC, payload, sizeof(payload));
}

static void SendImu(void)
{
    uint8_t payload[40] = {0};
    FLOAT_Angle angle;
    uint32_t sequence, stamp;

    if (!imu_copy_fresh(&angle, &sequence, &stamp)) return;
    if (sequence == last_imu_sequence) return;
    last_imu_sequence = sequence;


    AuvProtocol_WriteU32Le(&payload[0], sequence);
    AuvProtocol_WriteF32Le(&payload[4], angle.rol * AUV_DEGREES_TO_RADIANS);
    AuvProtocol_WriteF32Le(&payload[8], angle.pit * AUV_DEGREES_TO_RADIANS);
    AuvProtocol_WriteF32Le(&payload[12], angle.yaw * AUV_DEGREES_TO_RADIANS);
    /* H30 firmware currently exposes Euler angles only. NaN means unavailable. */
    AuvProtocol_WriteU32Le(&payload[16], AUV_QUIET_NAN_BITS);
    AuvProtocol_WriteU32Le(&payload[20], AUV_QUIET_NAN_BITS);
    AuvProtocol_WriteU32Le(&payload[24], AUV_QUIET_NAN_BITS);
    AuvProtocol_WriteU32Le(&payload[28], AUV_QUIET_NAN_BITS);
    AuvProtocol_WriteU32Le(&payload[32], AUV_QUIET_NAN_BITS);
    AuvProtocol_WriteU32Le(&payload[36], AUV_QUIET_NAN_BITS);
    SendFrame(AUV_MSG_IMU, payload, sizeof(payload));
}

static void SendDepth(uint32_t now_ms)
{
    uint8_t payload[9] = {0};
    AuvDepthSample sample;
    uint32_t mask = __get_PRIMASK();
    uint8_t fresh;
    __disable_irq();
    now_ms = HAL_GetTick();
    fresh = AuvDepth_CopyFresh(now_ms, &sample);
    __set_PRIMASK(mask);
    AuvProtocol_WriteU32Le(&payload[0], depth_sequence++);
    if (fresh != 0U)
        AuvProtocol_WriteF32Le(&payload[4], sample.depth_m);
    else
        AuvProtocol_WriteU32Le(&payload[4], AUV_QUIET_NAN_BITS);
    payload[8] = fresh;
    SendFrame(AUV_MSG_DEPTH, payload, sizeof(payload));
}

static void SendActuatorStatus(void)
{
    uint8_t payload[16] = {0};
    AuvGripperStatus status;
    AuvGripper_GetStatus(&status);
    AuvProtocol_WriteU32Le(&payload[0], actuator_sequence++);
    payload[4] = AUV_GRIPPER_ACTUATOR_ID;
    payload[5] = (uint8_t)status.state;
    payload[6] = (status.calibrated != 0U) ? 1U : 0U;
    AuvProtocol_WriteU16Le(&payload[7], status.current_us);
    AuvProtocol_WriteU16Le(&payload[9], status.target_us);
    payload[11] = status.error_flags;
    AuvProtocol_WriteU32Le(&payload[12], status.last_command_sequence);
    SendFrame(AUV_MSG_ACTUATOR_STATUS, payload, sizeof(payload));
}

void AuvLink_Init(void)
{
    UART_HandleTypeDef *uart = LinkUart();
    AuvSafety_Init(HAL_GetTick());
    AuvMotionTarget_Init();
    AuvDepth_Init();
    AuvProtocolParser_Init(&parser);
    ack_pending = 0U;
    status_sequence = 0U;
    level_ack_pending = 0U;
    level_sequence_seen = 0U;
    last_status_ms = HAL_GetTick();
    last_depth_ms = last_status_ms;
    last_imu_ms = last_status_ms;
    last_imu_sequence = imu_sample_sequence;

    external_safety_seen = AUV_ROV_MANUAL_TRIAL ? 1U : 0U;
    external_kill_active = AUV_ROV_MANUAL_TRIAL ? 1U : 0U;
    external_sensors_valid = AUV_ROV_MANUAL_TRIAL ? 1U : 0U;
    depth_sequence = 0U;
    actuator_sequence = 0U;
    last_actuator_ms = last_status_ms;
    {
        const AuvGripperConfig gripper_config = {
            AUV_GRIPPER_CALIBRATED,
            AUV_GRIPPER_MIN_US,
            AUV_GRIPPER_NEUTRAL_US,
            AUV_GRIPPER_MAX_US,
            AUV_GRIPPER_CLOSE_US,
            AUV_GRIPPER_OPEN_US,
            AUV_GRIPPER_SLEW_US_PER_TICK
        };
        const AuvGripperBootTestConfig boot_test_config = {
            AUV_GRIPPER_BOOT_TEST_ENABLED, AUV_GRIPPER_BOOT_START_US,
            AUV_GRIPPER_BOOT_EXCURSION_US, AUV_GRIPPER_BOOT_WAIT_MS,
            AUV_GRIPPER_BOOT_HOLD_MS
        };
        AuvGripper_Init(&gripper_config);
        (void)AuvGripper_ConfigureBootTest(&boot_test_config, HAL_GetTick());
    }

    (void)HAL_UART_DeInit(uart);
    uart->Init.BaudRate = AUV_LINK_BAUD_RATE;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(uart) != HAL_OK) Error_Handler();
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1U) != HAL_OK) Error_Handler();
}

void AuvLink_RxComplete(UART_HandleTypeDef *huart)
{
    AuvProtocolFrame frame;
    uint8_t byte;
    if (huart != LinkUart()) return;
    byte = rx_byte;
    (void)HAL_UART_Receive_IT(LinkUart(), &rx_byte, 1U);
    if (AuvProtocolParser_Push(&parser, byte, &frame) == AUV_PARSE_FRAME_READY)
        DispatchFrame(&frame, HAL_GetTick());
}

void AuvLink_RxError(UART_HandleTypeDef *huart)
{
    if (huart != LinkUart()) return;
    AuvProtocolParser_Init(&parser);
    (void)AuvSafety_RequestArm(0U, HAL_GetTick());
    if (LinkUart()->RxState == HAL_UART_STATE_READY)
        (void)HAL_UART_Receive_IT(LinkUart(), &rx_byte, 1U);
}

void AuvLink_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t primask = __get_PRIMASK();
    AuvDepthSample depth;
    __disable_irq();
    now_ms = HAL_GetTick();
    if (LinkUart()->RxState == HAL_UART_STATE_READY) {
        AuvProtocolParser_Init(&parser);
        (void)AuvSafety_RequestArm(0U, now_ms);
        (void)HAL_UART_Receive_IT(LinkUart(), &rx_byte, 1U);
    }
    const uint8_t depth_fresh = AuvDepth_CopyFresh(now_ms, &depth);
    const uint8_t imu_fresh = (imu_sample_sequence != 0U) &&
        ((uint32_t)(now_ms - imu_last_sample_ms) <= AUV_IMU_TIMEOUT_MS);
    AuvSafety_SetInputs(external_kill_active,
        (external_safety_seen != 0U && external_sensors_valid != 0U &&
         (AUV_ROV_MANUAL_TRIAL || depth_fresh != 0U) && imu_fresh != 0U && imu_level_calibrated) ? 1U : 0U, now_ms);
    __set_PRIMASK(primask);
#if AUV_ROV_MANUAL_TRIAL && (AUV_ROV_TRIAL_ARM_MAX_MS > 0U)
    if (AuvSafety_IsArmed() &&
        (uint32_t)(now_ms-trial_arm_start_ms) >= AUV_ROV_TRIAL_ARM_MAX_MS) {
        external_kill_active = 1U;
        (void)AuvSafety_RequestArm(0U,now_ms);
    }
#endif
    SendPendingAck();
    if ((uint32_t)(now_ms - last_imu_ms) >= AUV_IMU_PERIOD_MS) {
        last_imu_ms = now_ms;
        SendImu();
    }
    if ((uint32_t)(now_ms - last_status_ms) >= AUV_STATUS_PERIOD_MS) {
        last_status_ms = now_ms;
        SendStatus(now_ms);
        SendPidDiagnostic();
    }
    if ((uint32_t)(now_ms - last_depth_ms) >= AUV_DEPTH_PERIOD_MS) {
        last_depth_ms = now_ms;
        SendDepth(now_ms);
    }
    if ((uint32_t)(now_ms - last_actuator_ms) >= AUV_ACTUATOR_PERIOD_MS) {
        last_actuator_ms = now_ms;
        SendActuatorStatus();
    }
}

void AuvLink_SetSafetyInputs(uint8_t kill_active,
                             uint8_t sensors_valid)
{
    external_kill_active = (kill_active != 0U) ? 1U : 0U;
    external_sensors_valid = (sensors_valid != 0U) ? 1U : 0U;
    external_safety_seen = 1U;
}

uint8_t AuvLink_UpdateDepth(float depth_m)
{
    return AuvDepth_Update(depth_m, HAL_GetTick());
}

void AuvLink_InvalidateDepth(void)
{
    AuvDepth_Invalidate(HAL_GetTick());
}
