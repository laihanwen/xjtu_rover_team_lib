/**
 * @file AuvLink.c
 * @brief Raspberry Pi UART link and safety integration.
 */

#include "AuvLink.h"

#include "AuvProtocol.h"
#include "AuvSafety.h"
#include "imu.h"
#include "usart.h"

#ifndef AUV_LINK_UART_HANDLE
#define AUV_LINK_UART_HANDLE huart3
#endif

#define AUV_LINK_BAUD_RATE          115200U
#define AUV_STATUS_PERIOD_MS        100U
#define AUV_IMU_PERIOD_MS           20U
#define AUV_DEPTH_PERIOD_MS         100U
#define AUV_DEGREES_TO_RADIANS      0.01745329251994329577f
#define AUV_QUIET_NAN_BITS          0x7FC00000UL
#define AUV_ERROR_HEARTBEAT_TIMEOUT (1UL << 0)
#define AUV_ERROR_SENSOR_INVALID    (1UL << 1)
#define AUV_ERROR_LEAK              (1UL << 2)
#define AUV_ERROR_KILL              (1UL << 3)

static AuvProtocolParser parser;
static uint8_t rx_byte;
static volatile uint8_t ack_pending;
static volatile uint8_t ack_type;
static volatile uint8_t ack_result;
static volatile uint32_t ack_sequence;
static uint32_t status_sequence;
static uint32_t last_status_ms;
static uint32_t last_depth_ms;
static uint32_t last_imu_ms;
static uint32_t last_imu_sequence;
static uint32_t depth_sequence;

static UART_HandleTypeDef *LinkUart(void)
{
    return &AUV_LINK_UART_HANDLE;
}

static void QueueAck(uint8_t type, uint8_t result, uint32_t sequence)
{
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
            result = AuvSafety_RequestArm(frame->payload[4], now_ms);
        }
        QueueAck(AUV_MSG_SET_ARMED, (uint8_t)result, sequence);
    } else if ((frame->message_type == AUV_MSG_MOTION_TARGET) ||
               (frame->message_type == AUV_MSG_ACTUATOR_COMMAND)) {
        uint32_t sequence = (frame->payload_length >= 4U)
            ? AuvProtocol_ReadU32Le(frame->payload) : 0U;
        QueueAck(frame->message_type, AUV_ARM_UNSUPPORTED, sequence);
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

    if (ack_pending == 0U) return;
    __disable_irq();
    type = ack_type;
    result = ack_result;
    sequence = ack_sequence;
    ack_pending = 0U;
    __enable_irq();
    payload[0] = type;
    payload[1] = result;
    AuvProtocol_WriteU32Le(&payload[2], sequence);
    SendFrame(AUV_MSG_ACK, payload, sizeof(payload));
}

static void SendStatus(uint32_t now_ms)
{
    const AuvSafetyContext *context = AuvSafety_GetContext();
    uint8_t payload[30] = {0};
    uint8_t state_flags = 0U;
    uint32_t error_flags = 0U;

    if (context->state == AUV_SAFETY_ARMED) state_flags |= 1U << 0;
    if (context->leak_detected != 0U) state_flags |= 1U << 1;
    if (context->state == AUV_SAFETY_FAILSAFE) state_flags |= 1U << 2;
    if ((context->heartbeat_seen != 0U) &&
        ((uint32_t)(now_ms - context->last_heartbeat_ms) >
         AUV_HEARTBEAT_TIMEOUT_MS)) error_flags |= AUV_ERROR_HEARTBEAT_TIMEOUT;
    if (context->sensors_valid == 0U) error_flags |= AUV_ERROR_SENSOR_INVALID;
    if (context->leak_detected != 0U) error_flags |= AUV_ERROR_LEAK;
    if (context->kill_active != 0U) error_flags |= AUV_ERROR_KILL;

    AuvProtocol_WriteU32Le(&payload[0], status_sequence++);
    payload[4] = state_flags;
    AuvProtocol_WriteU32Le(&payload[5], error_flags);
    AuvProtocol_WriteU32Le(&payload[9], AUV_QUIET_NAN_BITS);  /* voltage unavailable */
    AuvProtocol_WriteU32Le(&payload[13], AUV_QUIET_NAN_BITS); /* depth unavailable */
    payload[29] = 0U; /* P5 will publish measured thruster outputs. */
    SendFrame(AUV_MSG_STATUS, payload, sizeof(payload));
}

static void SendImu(void)
{
    uint8_t payload[40] = {0};
    FLOAT_Angle angle;
    uint32_t sequence;

    __disable_irq();
    sequence = imu_sample_sequence;
    angle = Angle_Measure;
    __enable_irq();
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

static void SendDepthUnavailable(void)
{
    uint8_t payload[9] = {0};
    AuvProtocol_WriteU32Le(&payload[0], depth_sequence++);
    AuvProtocol_WriteU32Le(&payload[4], AUV_QUIET_NAN_BITS);
    payload[8] = 0U;
    SendFrame(AUV_MSG_DEPTH, payload, sizeof(payload));
}

void AuvLink_Init(void)
{
    UART_HandleTypeDef *uart = LinkUart();
    AuvSafety_Init(HAL_GetTick());
    AuvProtocolParser_Init(&parser);
    ack_pending = 0U;
    status_sequence = 0U;
    last_status_ms = HAL_GetTick();
    last_depth_ms = last_status_ms;
    last_imu_ms = last_status_ms;
    last_imu_sequence = imu_sample_sequence;
    depth_sequence = 0U;

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
    if (huart != LinkUart()) return;
    if (AuvProtocolParser_Push(&parser, rx_byte, &frame) == AUV_PARSE_FRAME_READY)
        DispatchFrame(&frame, HAL_GetTick());
    (void)HAL_UART_Receive_IT(LinkUart(), &rx_byte, 1U);
}

void AuvLink_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    AuvSafety_Tick(now_ms);
    SendPendingAck();
    if ((uint32_t)(now_ms - last_imu_ms) >= AUV_IMU_PERIOD_MS) {
        last_imu_ms = now_ms;
        SendImu();
    }
    if ((uint32_t)(now_ms - last_status_ms) >= AUV_STATUS_PERIOD_MS) {
        last_status_ms = now_ms;
        SendStatus(now_ms);
    }
    if ((uint32_t)(now_ms - last_depth_ms) >= AUV_DEPTH_PERIOD_MS) {
        last_depth_ms = now_ms;
        SendDepthUnavailable();
    }
}

void AuvLink_SetSafetyInputs(uint8_t leak_detected,
                             uint8_t kill_active,
                             uint8_t sensors_valid)
{
    AuvSafety_SetInputs(leak_detected, kill_active, sensors_valid, HAL_GetTick());
}
