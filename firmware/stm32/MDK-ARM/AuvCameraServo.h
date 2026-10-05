/**
 * @file AuvCameraServo.h
 * @brief Bounded camera-servo state machine for PC7/TIM8_CH2.
 */

#ifndef AUV_CAMERA_SERVO_H
#define AUV_CAMERA_SERVO_H

#include <stdint.h>

#include "AuvSafety.h"

#define AUV_CAMERA_SERVO_ACTUATOR_ID 2U
#define AUV_CAMERA_SERVO_ERROR_UNCALIBRATED (1U << 0)
#define AUV_CAMERA_SERVO_ERROR_CONFIG       (1U << 1)
#define AUV_CAMERA_SERVO_ERROR_SEQUENCE     (1U << 2)

typedef enum {
    AUV_CAMERA_SERVO_UNCALIBRATED = 0,
    AUV_CAMERA_SERVO_STOPPED = 1,
    AUV_CAMERA_SERVO_MOVING = 2,
    AUV_CAMERA_SERVO_POSITIONED = 3,
    AUV_CAMERA_SERVO_FAULT = 4
} AuvCameraServoState;

typedef struct {
    uint8_t calibrated;
    uint16_t minimum_ccr;
    uint16_t maximum_ccr;
    uint16_t startup_ccr;
    uint16_t slew_per_tick;
} AuvCameraServoConfig;

typedef struct {
    AuvCameraServoState state;
    uint16_t current_ccr;
    uint16_t target_ccr;
    uint8_t error_flags;
    uint32_t last_command_sequence;
} AuvCameraServoStatus;

void AuvCameraServo_Init(const AuvCameraServoConfig *config);
AuvArmResult AuvCameraServo_Accept(const uint8_t *payload,
                                   uint8_t payload_length,
                                   uint8_t armed,
                                   uint8_t pi_source_authorized);
void AuvCameraServo_CommandRemote(uint8_t dial, uint8_t enabled, uint8_t armed);
void AuvCameraServo_Tick(uint8_t armed);
uint16_t AuvCameraServo_GetCcr(void);
const AuvCameraServoStatus *AuvCameraServo_GetStatus(void);

#endif
