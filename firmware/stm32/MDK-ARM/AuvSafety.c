/**
 * @file AuvSafety.c
 * @brief Explicit arm/disarm and heartbeat failsafe state machine.
 */

#include "AuvSafety.h"

static AuvSafetyContext safety;

static uint8_t InputsSafe(void)
{
    return ((safety.leak_detected == 0U) &&
            (safety.kill_active == 0U) &&
            (safety.sensors_valid != 0U)) ? 1U : 0U;
}

static uint8_t HeartbeatFresh(uint32_t now_ms)
{
    return ((safety.heartbeat_seen != 0U) &&
            ((uint32_t)(now_ms - safety.last_heartbeat_ms) <=
             AUV_HEARTBEAT_TIMEOUT_MS)) ? 1U : 0U;
}

void AuvSafety_Init(uint32_t now_ms)
{
    safety.state = AUV_SAFETY_DISARMED;
    safety.last_heartbeat_ms = now_ms;
    safety.heartbeat_seen = 0U;
    safety.leak_detected = 0U;
    safety.kill_active = 0U;
    safety.sensors_valid = 0U;
}

void AuvSafety_OnHeartbeat(uint32_t now_ms)
{
    safety.last_heartbeat_ms = now_ms;
    safety.heartbeat_seen = 1U;
    if ((safety.state == AUV_SAFETY_FAILSAFE) && (InputsSafe() != 0U))
        safety.state = AUV_SAFETY_DISARMED;
}

void AuvSafety_Tick(uint32_t now_ms)
{
    if ((InputsSafe() == 0U) ||
        ((safety.heartbeat_seen != 0U) && (HeartbeatFresh(now_ms) == 0U)))
        safety.state = AUV_SAFETY_FAILSAFE;
}

AuvArmResult AuvSafety_RequestArm(uint8_t arm, uint32_t now_ms)
{
    if (arm > 1U) return AUV_ARM_MALFORMED;
    if (arm == 0U) {
        safety.state = AUV_SAFETY_DISARMED;
        return AUV_ARM_ACCEPTED;
    }
    if ((safety.state != AUV_SAFETY_DISARMED) ||
        (InputsSafe() == 0U) || (HeartbeatFresh(now_ms) == 0U))
        return AUV_ARM_UNSAFE;
    safety.state = AUV_SAFETY_ARMED;
    return AUV_ARM_ACCEPTED;
}

void AuvSafety_SetInputs(uint8_t leak_detected,
                         uint8_t kill_active,
                         uint8_t sensors_valid,
                         uint32_t now_ms)
{
    safety.leak_detected = (leak_detected != 0U) ? 1U : 0U;
    safety.kill_active = (kill_active != 0U) ? 1U : 0U;
    safety.sensors_valid = (sensors_valid != 0U) ? 1U : 0U;
    AuvSafety_Tick(now_ms);
}

AuvSafetyState AuvSafety_GetState(void)
{
    return safety.state;
}

uint8_t AuvSafety_IsArmed(void)
{
    return (safety.state == AUV_SAFETY_ARMED) ? 1U : 0U;
}

const AuvSafetyContext *AuvSafety_GetContext(void)
{
    return &safety;
}
