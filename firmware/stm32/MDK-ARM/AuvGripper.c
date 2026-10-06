/**
 * @file AuvGripper.c
 * @brief Allocation-free, hardware-independent single-servo gripper control.
 */

#include "AuvGripper.h"

#include <stddef.h>

#include "AuvGripperConfig.h"
#include "AuvProtocol.h"

static AuvGripperConfig active_config;
static volatile AuvGripperStatus active_status;
static volatile uint8_t command_seen;
static volatile uint8_t initialized;
static uint8_t last_remote_action;
static AuvGripperBootTestConfig boot_test;
static uint32_t boot_phase_ms;
static uint8_t boot_test_configured;

static uint8_t BootTestActive(void)
{
    return active_status.boot_test_state >= AUV_GRIPPER_TEST_WAIT &&
        active_status.boot_test_state <= AUV_GRIPPER_TEST_RETURN;
}

static uint8_t SequenceNewer(uint32_t incoming, uint32_t previous)
{
    return ((int32_t)(incoming - previous) > 0) ? 1U : 0U;
}

static uint8_t ConfigValid(const AuvGripperConfig *config)
{
    if (config == NULL) return 0U;
    if ((config->minimum_us >= config->maximum_us) ||
        (config->neutral_us < config->minimum_us) ||
        (config->neutral_us > config->maximum_us) ||
        (config->close_us < config->minimum_us) ||
        (config->close_us > config->maximum_us) ||
        (config->open_us < config->minimum_us) ||
        (config->open_us > config->maximum_us) ||
        (config->close_us >= config->open_us) ||
        (config->slew_us_per_tick == 0U)) return 0U;
    return 1U;
}

static AuvArmResult ApplyAction(AuvGripperAction action, uint8_t armed)
{
    if (action == AUV_GRIPPER_ACTION_STOP) {
        AuvGripper_CancelBootTest();
        active_status.target_us = active_status.current_us;
        active_status.state = (active_status.calibrated != 0U)
            ? AUV_GRIPPER_STOPPED : AUV_GRIPPER_UNCALIBRATED;
        return AUV_ARM_ACCEPTED;
    }
    if (BootTestActive()) return AUV_ARM_UNSAFE;
    if (armed == 0U) return AUV_ARM_DISARMED;
    if (active_status.calibrated == 0U) return AUV_ARM_UNSAFE;
    if (active_status.state == AUV_GRIPPER_FAULT) return AUV_ARM_UNSAFE;

    if (action == AUV_GRIPPER_ACTION_CLOSE) {
        active_status.target_us = active_config.close_us;
        active_status.state = AUV_GRIPPER_MOVING_CLOSE;
        return AUV_ARM_ACCEPTED;
    }
    if (action == AUV_GRIPPER_ACTION_OPEN) {
        active_status.target_us = active_config.open_us;
        active_status.state = AUV_GRIPPER_MOVING_OPEN;
        return AUV_ARM_ACCEPTED;
    }
    return AUV_ARM_MALFORMED;
}

void AuvGripper_Init(const AuvGripperConfig *config)
{
    initialized = 0U;
    active_status.calibrated = 0U;
    active_status.state = AUV_GRIPPER_UNCALIBRATED;
    active_status.current_us = AUV_GRIPPER_NEUTRAL_US;
    active_status.target_us = AUV_GRIPPER_NEUTRAL_US;
    active_status.error_flags = AUV_GRIPPER_ERROR_UNCALIBRATED;
    active_status.last_command_sequence = 0U;
    command_seen = 0U;
    last_remote_action = 0xFFU;
    boot_test_configured = 0U;
    active_status.boot_test_state = AUV_GRIPPER_TEST_DISABLED;

    if (ConfigValid(config) == 0U) {
        active_status.state = AUV_GRIPPER_FAULT;
        active_status.calibrated = 0U;
        active_status.error_flags |= AUV_GRIPPER_ERROR_CONFIG;
        initialized = 1U;
        return;
    }
    active_config = *config;
    active_status.current_us = active_config.neutral_us;
    active_status.target_us = active_config.neutral_us;
    initialized = 1U;
    if (active_config.calibrated == 0U) return;

    active_status.calibrated = 1U;
    active_status.error_flags = 0U;
    /* Calibration does not authorize an automatic full-close boot movement. */
    active_status.state = AUV_GRIPPER_STOPPED;
}

uint8_t AuvGripper_PwmEnabled(void)
{
    return initialized && active_status.calibrated &&
        active_status.state != AUV_GRIPPER_FAULT;
}

uint8_t AuvGripper_ConfigureBootTest(const AuvGripperBootTestConfig *config,
                                    uint32_t now_ms)
{
    if (config == NULL || boot_test_configured) return 0U;
    boot_test_configured = 1U;
    if (!config->enabled) return 1U;
    if (!AuvGripper_PwmEnabled() ||
        config->start_us < active_config.close_us ||
        config->start_us > active_config.open_us ||
        config->excursion_us < active_config.close_us ||
        config->excursion_us > active_config.open_us ||
        config->start_us == config->excursion_us) {
        active_status.state = AUV_GRIPPER_FAULT;
        active_status.calibrated = 0U;
        active_status.error_flags |= AUV_GRIPPER_ERROR_CONFIG;
        return 0U;
    }
    boot_test = *config;
    boot_phase_ms = now_ms;
    active_status.current_us = config->start_us;
    active_status.target_us = config->start_us;
    active_status.boot_test_state = AUV_GRIPPER_TEST_WAIT;
    return 1U;
}

void AuvGripper_CancelBootTest(void)
{
    if (!BootTestActive()) return;
    active_status.target_us = active_status.current_us;
    active_status.state = AUV_GRIPPER_STOPPED;
    active_status.boot_test_state = AUV_GRIPPER_TEST_CANCELLED;
}

void AuvGripper_BootTestTick(uint32_t now_ms)
{
    switch (active_status.boot_test_state) {
    case AUV_GRIPPER_TEST_WAIT:
        if ((uint32_t)(now_ms - boot_phase_ms) >= boot_test.startup_wait_ms) {
            active_status.target_us = boot_test.excursion_us;
            active_status.state = (boot_test.excursion_us > boot_test.start_us)
                ? AUV_GRIPPER_MOVING_OPEN : AUV_GRIPPER_MOVING_CLOSE;
            active_status.boot_test_state = AUV_GRIPPER_TEST_OUT;
        }
        break;
    case AUV_GRIPPER_TEST_OUT:
        if (active_status.current_us == active_status.target_us) {
            boot_phase_ms = now_ms;
            active_status.boot_test_state = AUV_GRIPPER_TEST_HOLD;
        }
        break;
    case AUV_GRIPPER_TEST_HOLD:
        if ((uint32_t)(now_ms - boot_phase_ms) >= boot_test.endpoint_hold_ms) {
            active_status.target_us = boot_test.start_us;
            active_status.state = (boot_test.start_us > boot_test.excursion_us)
                ? AUV_GRIPPER_MOVING_OPEN : AUV_GRIPPER_MOVING_CLOSE;
            active_status.boot_test_state = AUV_GRIPPER_TEST_RETURN;
        }
        break;
    case AUV_GRIPPER_TEST_RETURN:
        if (active_status.current_us == active_status.target_us)
            active_status.boot_test_state = AUV_GRIPPER_TEST_COMPLETE;
        break;
    default: break;
    }
}

AuvArmResult AuvGripper_Accept(const uint8_t *payload,
                               uint8_t payload_length,
                               uint8_t armed,
                               uint8_t pi_source_authorized)
{
    uint32_t sequence;
    float normalized;
    AuvGripperAction action;
    AuvArmResult result;

    if ((payload == NULL) || (payload_length != 9U)) return AUV_ARM_MALFORMED;
    sequence = AuvProtocol_ReadU32Le(&payload[0]);
    if (payload[4] != AUV_GRIPPER_ACTUATOR_ID) return AUV_ARM_UNSUPPORTED;
    normalized = AuvProtocol_ReadF32Le(&payload[5]);
    if (normalized != normalized) return AUV_ARM_MALFORMED;
    if ((command_seen != 0U) &&
        (SequenceNewer(sequence, active_status.last_command_sequence) == 0U)) {
        active_status.error_flags |= AUV_GRIPPER_ERROR_SEQUENCE;
        return AUV_ARM_MALFORMED;
    }
    if (normalized <= -0.5f) action = AUV_GRIPPER_ACTION_CLOSE;
    else if (normalized >= 0.5f) action = AUV_GRIPPER_ACTION_OPEN;
    else if ((normalized >= -0.1f) && (normalized <= 0.1f))
        action = AUV_GRIPPER_ACTION_STOP;
    else return AUV_ARM_MALFORMED;

    if ((action != AUV_GRIPPER_ACTION_STOP) &&
        (armed != 0U) && (pi_source_authorized == 0U)) return AUV_ARM_UNSAFE;

    active_status.last_command_sequence = sequence;
    command_seen = 1U;
    result = ApplyAction(action, armed);
    return result;
}

void AuvGripper_CommandRemote(uint8_t dial, uint8_t enabled, uint8_t armed)
{
    uint8_t requested = AUV_GRIPPER_ACTION_STOP;
    if ((enabled == 0U) || (armed == 0U) || BootTestActive()) {
        last_remote_action = 0xFFU;
        return;
    }
    if (dial <= AUV_GRIPPER_RC_CLOSE_MAX)
        requested = AUV_GRIPPER_ACTION_CLOSE;
    else if (dial >= AUV_GRIPPER_RC_OPEN_MIN)
        requested = AUV_GRIPPER_ACTION_OPEN;
    if (requested == last_remote_action) return;
    last_remote_action = requested;
    (void)ApplyAction((AuvGripperAction)requested, armed);
}

void AuvGripper_Tick(void)
{
    uint16_t current = active_status.current_us;
    uint16_t target = active_status.target_us;
    uint16_t step = active_config.slew_us_per_tick;

    if ((initialized == 0U) ||
        (active_status.calibrated == 0U) ||
        (active_status.state == AUV_GRIPPER_FAULT)) return;
    if (current < target) {
        uint16_t remaining = (uint16_t)(target - current);
        current = (remaining <= step) ? target : (uint16_t)(current + step);
    } else if (current > target) {
        uint16_t remaining = (uint16_t)(current - target);
        current = (remaining <= step) ? target : (uint16_t)(current - step);
    }
    active_status.current_us = current;
    if (current != target) return;
    if (target == active_config.close_us) active_status.state = AUV_GRIPPER_CLOSED;
    else if (target == active_config.open_us) active_status.state = AUV_GRIPPER_OPENED;
    else active_status.state = AUV_GRIPPER_STOPPED;
}

uint16_t AuvGripper_GetPulseUs(void)
{
    if (initialized == 0U) return AUV_GRIPPER_NEUTRAL_US;
    return active_status.current_us;
}

void AuvGripper_GetStatus(AuvGripperStatus *status)
{
    if (status == NULL) return;
    status->calibrated = active_status.calibrated;
    status->state = active_status.state;
    status->current_us = active_status.current_us;
    status->target_us = active_status.target_us;
    status->error_flags = active_status.error_flags;
    status->last_command_sequence = active_status.last_command_sequence;
    status->boot_test_state = active_status.boot_test_state;
}
