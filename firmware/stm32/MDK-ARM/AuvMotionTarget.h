/**
 * @file AuvMotionTarget.h
 * @brief Validated, timeout-protected Pi motion target storage.
 */

#ifndef AUV_MOTION_TARGET_H
#define AUV_MOTION_TARGET_H

#include <stdint.h>

#include "AuvSafety.h"

#define AUV_MOTION_TARGET_TIMEOUT_MS 250U

typedef struct {
    uint32_t sequence;
    float vx;
    float vy;
    float depth;
    float yaw;
} AuvMotionTarget;

void AuvMotionTarget_Init(void);
AuvArmResult AuvMotionTarget_Accept(const uint8_t *payload,
                                    uint8_t payload_length,
                                    uint32_t now_ms,
                                    uint8_t armed);
uint8_t AuvMotionTarget_CopyFresh(uint32_t now_ms, AuvMotionTarget *target);

#endif
