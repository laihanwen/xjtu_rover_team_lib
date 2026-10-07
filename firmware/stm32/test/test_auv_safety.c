#include "AuvSafety.h"

#include <assert.h>
#include <stdio.h>

static void MakeReady(uint32_t now_ms)
{
    AuvSafety_SetInputs(0U, 1U, now_ms);
    AuvSafety_OnHeartbeat(now_ms);
}

static void TestExplicitArmAndTimeout(void)
{
    AuvSafety_Init(0U);
    assert(AuvSafety_GetState() == AUV_SAFETY_DISARMED);
    assert(AuvSafety_RequestArm(1U, 0U) == AUV_ARM_UNSAFE);
    MakeReady(10U);
    assert(AuvSafety_GetState() == AUV_SAFETY_DISARMED);
    assert(AuvSafety_RequestArm(1U, 1999U) == AUV_ARM_UNSAFE);
    MakeReady(2000U);
    assert(AuvSafety_RequestArm(1U, 2000U) == AUV_ARM_ACCEPTED);
    assert(AuvSafety_IsArmed() != 0U);
    AuvSafety_Tick(2500U);
    assert(AuvSafety_IsArmed() != 0U);
    AuvSafety_Tick(2501U);
    assert(AuvSafety_GetState() == AUV_SAFETY_FAILSAFE);
    AuvSafety_OnHeartbeat(2502U);
    assert(AuvSafety_GetState() == AUV_SAFETY_DISARMED);
    assert(AuvSafety_IsArmed() == 0U);
}

static void TestFaultsAndDisarm(void)
{
    AuvSafety_Init(0U);
    MakeReady(2000U);
    assert(AuvSafety_RequestArm(1U, 2000U) == AUV_ARM_ACCEPTED);
    AuvSafety_SetInputs(1U, 1U, 2001U);
    assert(AuvSafety_GetState() == AUV_SAFETY_FAILSAFE);
    assert(AuvSafety_RequestArm(1U, 2001U) == AUV_ARM_UNSAFE);
    assert(AuvSafety_RequestArm(0U, 2001U) == AUV_ARM_ACCEPTED);
    assert(AuvSafety_GetState() == AUV_SAFETY_DISARMED);
    AuvSafety_SetInputs(1U, 1U, 2002U);
    assert(AuvSafety_GetState() == AUV_SAFETY_FAILSAFE);
    AuvSafety_SetInputs(0U, 0U, 2003U);
    assert(AuvSafety_GetState() == AUV_SAFETY_FAILSAFE);
}

static void TestTickRollover(void)
{
    AuvSafety_Init(0xFFFFFFF0U);
    MakeReady(0x000007C0U);
    assert(AuvSafety_RequestArm(1U, 0x000007C0U) == AUV_ARM_ACCEPTED);
    AuvSafety_OnHeartbeat(0xFFFFFFF0U);
    AuvSafety_Tick(0x00000010U);
    assert(AuvSafety_IsArmed() != 0U);
    AuvSafety_Tick(0x00000300U);
    assert(AuvSafety_GetState() == AUV_SAFETY_FAILSAFE);
}

static void TestHeartbeatCannotArm(void)
{
    AuvSafety_Init(0U);
    AuvSafety_SetInputs(0U, 1U, 0U);
    AuvSafety_OnHeartbeat(1U);
    assert(AuvSafety_GetState() == AUV_SAFETY_DISARMED);
    assert(AuvSafety_RequestArm(2U, 1U) == AUV_ARM_MALFORMED);
}

int main(void)
{
    TestExplicitArmAndTimeout();
    TestFaultsAndDisarm();
    TestTickRollover();
    TestHeartbeatCannotArm();
    puts("AuvSafety host tests passed");
    return 0;
}
