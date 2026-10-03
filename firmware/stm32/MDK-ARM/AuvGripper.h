/**
 * @file AuvGripper.h
 * @brief Portable state machine for the T35-L single-servo gripper.
 */

#ifndef AUV_GRIPPER_H
#define AUV_GRIPPER_H

#include <stdint.h>

#include "AuvSafety.h"

#define AUV_GRIPPER_ACTUATOR_ID 1U
#define AUV_GRIPPER_ERROR_UNCALIBRATED (1U << 0)
#define AUV_GRIPPER_ERROR_CONFIG       (1U << 1)
#define AUV_GRIPPER_ERROR_SEQUENCE     (1U << 2)

typedef enum {
    AUV_GRIPPER_ACTION_STOP = 0,
    AUV_GRIPPER_ACTION_CLOSE = 1,
    AUV_GRIPPER_ACTION_OPEN = 2
} AuvGripperAction;

typedef enum {
    AUV_GRIPPER_UNCALIBRATED = 0,
    AUV_GRIPPER_STOPPED = 1,
    AUV_GRIPPER_MOVING_CLOSE = 2,
    AUV_GRIPPER_CLOSED = 3,
    AUV_GRIPPER_MOVING_OPEN = 4,
    AUV_GRIPPER_OPENED = 5,
    AUV_GRIPPER_FAULT = 6
} AuvGripperState;

typedef struct {
    uint8_t calibrated;
    uint16_t minimum_us;
    uint16_t neutral_us;
    uint16_t maximum_us;
    uint16_t close_us;
    uint16_t open_us;
    uint16_t slew_us_per_tick;
} AuvGripperConfig;

typedef struct {
    uint8_t calibrated;
    AuvGripperState state;
    uint16_t current_us;
    uint16_t target_us;
    uint8_t error_flags;
    uint32_t last_command_sequence;
} AuvGripperStatus;

void AuvGripper_Init(const AuvGripperConfig *config);
AuvArmResult AuvGripper_Accept(const uint8_t *payload,
                               uint8_t payload_length,
                               uint8_t armed,
                               uint8_t pi_source_authorized);
void AuvGripper_CommandRemote(uint8_t dial, uint8_t enabled, uint8_t armed);
void AuvGripper_Tick(void);
uint16_t AuvGripper_GetPulseUs(void);
void AuvGripper_GetStatus(AuvGripperStatus *status);

#endif
