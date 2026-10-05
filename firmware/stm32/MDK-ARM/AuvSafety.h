/**
 * @file AuvSafety.h
 * @brief Explicit arm/disarm and heartbeat failsafe state machine.
 */

#ifndef AUV_SAFETY_H
#define AUV_SAFETY_H

#include <stdint.h>

#define AUV_HEARTBEAT_TIMEOUT_MS 500U
#define AUV_ARM_STARTUP_INHIBIT_MS 2000U

typedef enum {
    AUV_SAFETY_BOOT = 0,
    AUV_SAFETY_DISARMED,
    AUV_SAFETY_ARMED,
    AUV_SAFETY_FAILSAFE
} AuvSafetyState;

typedef enum {
    AUV_ARM_ACCEPTED = 0,
    AUV_ARM_MALFORMED = 1,
    AUV_ARM_DISARMED = 2,
    AUV_ARM_UNSAFE = 3,
    AUV_ARM_UNSUPPORTED = 4
} AuvArmResult;

typedef struct {
    volatile AuvSafetyState state;
    volatile uint32_t last_heartbeat_ms;
    volatile uint32_t boot_ms;
    volatile uint8_t heartbeat_seen;
    volatile uint8_t leak_detected;
    volatile uint8_t kill_active;
    volatile uint8_t sensors_valid;
} AuvSafetyContext;

void AuvSafety_Init(uint32_t now_ms);
void AuvSafety_OnHeartbeat(uint32_t now_ms);
void AuvSafety_Tick(uint32_t now_ms);
AuvArmResult AuvSafety_RequestArm(uint8_t arm, uint32_t now_ms);
void AuvSafety_SetInputs(uint8_t leak_detected,
                         uint8_t kill_active,
                         uint8_t sensors_valid,
                         uint32_t now_ms);
AuvSafetyState AuvSafety_GetState(void);
uint8_t AuvSafety_IsArmed(void);
const AuvSafetyContext *AuvSafety_GetContext(void);

#endif
