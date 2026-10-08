/**
 * @file AuvCameraServo.c
 * @brief Calibrated, rate-limited camera-servo control.
 */

#include "AuvCameraServo.h"

#include <stddef.h>

#include "AuvProtocol.h"

static AuvCameraServoConfig active_config;
static AuvCameraServoStatus active_status;
static uint8_t command_seen;

static uint8_t SequenceNewer(uint32_t sequence, uint32_t previous)
{
    return ((int32_t)(sequence - previous) > 0) ? 1U : 0U;
}

static uint8_t ConfigValid(const AuvCameraServoConfig *config)
{
    return (config != NULL && config->minimum_ccr < config->maximum_ccr &&
            config->startup_ccr >= config->minimum_ccr &&
            config->startup_ccr <= config->maximum_ccr &&
            config->slew_per_tick > 0U) ? 1U : 0U;
}

static uint16_t PositionFromNormalized(float normalized)
{
    const float unit = (normalized + 1.0f) * 0.5f;
    const float span = (float)(active_config.maximum_ccr - active_config.minimum_ccr);
    return (uint16_t)((float)active_config.minimum_ccr + unit * span + 0.5f);
}

void AuvCameraServo_Init(const AuvCameraServoConfig *config)
{
    active_status.state = AUV_CAMERA_SERVO_FAULT;
    active_status.current_ccr = 0U;
    active_status.target_ccr = 0U;
    active_status.error_flags = AUV_CAMERA_SERVO_ERROR_CONFIG;
    active_status.last_command_sequence = 0U;
    command_seen = 0U;
    if (ConfigValid(config) == 0U) return;
    active_config = *config;
    active_status.current_ccr = active_config.startup_ccr;
    active_status.target_ccr = active_config.startup_ccr;
    active_status.error_flags = 0U;
    if (active_config.calibrated == 0U) {
        active_status.state = AUV_CAMERA_SERVO_UNCALIBRATED;
        active_status.error_flags = AUV_CAMERA_SERVO_ERROR_UNCALIBRATED;
    } else {
        active_status.state = AUV_CAMERA_SERVO_STOPPED;
    }
}

AuvArmResult AuvCameraServo_Accept(const uint8_t *payload,
                                   uint8_t payload_length,
                                   uint8_t armed,
                                   uint8_t pi_source_authorized)
{
    uint32_t sequence;
    float normalized;
    if (payload == NULL || payload_length != 9U) return AUV_ARM_MALFORMED;
    sequence = AuvProtocol_ReadU32Le(payload);
    if (payload[4] != AUV_CAMERA_SERVO_ACTUATOR_ID) return AUV_ARM_UNSUPPORTED;
    normalized = AuvProtocol_ReadF32Le(&payload[5]);
    if (normalized != normalized || normalized < -1.0f || normalized > 1.0f)
        return AUV_ARM_MALFORMED;
    if (command_seen != 0U && SequenceNewer(sequence,active_status.last_command_sequence) == 0U) {
        active_status.error_flags |= AUV_CAMERA_SERVO_ERROR_SEQUENCE;
        return AUV_ARM_MALFORMED;
    }
    if (armed == 0U) return AUV_ARM_DISARMED;
    if (active_config.calibrated == 0U || pi_source_authorized == 0U)
        return AUV_ARM_UNSAFE;
    active_status.last_command_sequence = sequence;
    command_seen = 1U;
    active_status.target_ccr = PositionFromNormalized(normalized);
    active_status.state = (active_status.current_ccr == active_status.target_ccr) ?
        AUV_CAMERA_SERVO_POSITIONED : AUV_CAMERA_SERVO_MOVING;
    return AUV_ARM_ACCEPTED;
}

void AuvCameraServo_CommandRemote(uint8_t dial, uint8_t enabled, uint8_t armed)
{
    uint32_t span;
    if (active_config.calibrated == 0U || active_status.state == AUV_CAMERA_SERVO_FAULT) return;
    if (enabled == 0U || armed == 0U) {
        /* Deselect/disarm cancels the outstanding ramp, holding current PWM. */
        active_status.target_ccr = active_status.current_ccr;
        active_status.state = AUV_CAMERA_SERVO_STOPPED;
        return;
    }
    span = (uint32_t)(active_config.maximum_ccr - active_config.minimum_ccr);
    active_status.target_ccr = (uint16_t)(active_config.minimum_ccr +
        ((uint32_t)dial * span) / 255U);
    active_status.state = (active_status.current_ccr == active_status.target_ccr) ?
        AUV_CAMERA_SERVO_POSITIONED : AUV_CAMERA_SERVO_MOVING;
}

void AuvCameraServo_Tick(uint8_t armed)
{
    if (active_status.state == AUV_CAMERA_SERVO_FAULT ||
        active_status.state == AUV_CAMERA_SERVO_UNCALIBRATED) return;
    if (armed == 0U) {
        active_status.target_ccr = active_status.current_ccr;
        active_status.state = AUV_CAMERA_SERVO_STOPPED;
        return;
    }
    if (active_status.current_ccr < active_status.target_ccr) {
        uint32_t next = (uint32_t)active_status.current_ccr + active_config.slew_per_tick;
        active_status.current_ccr = (next > active_status.target_ccr) ?
            active_status.target_ccr : (uint16_t)next;
    } else if (active_status.current_ccr > active_status.target_ccr) {
        const uint16_t delta = active_status.current_ccr - active_status.target_ccr;
        active_status.current_ccr = (delta <= active_config.slew_per_tick) ?
            active_status.target_ccr :
            (uint16_t)(active_status.current_ccr - active_config.slew_per_tick);
    }
    active_status.state = (active_status.current_ccr == active_status.target_ccr) ?
        AUV_CAMERA_SERVO_POSITIONED : AUV_CAMERA_SERVO_MOVING;
}

uint16_t AuvCameraServo_GetCcr(void)
{
    return active_status.current_ccr;
}

const AuvCameraServoStatus *AuvCameraServo_GetStatus(void)
{
    return &active_status;
}
