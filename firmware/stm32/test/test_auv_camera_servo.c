#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "AuvCameraServo.h"
#include "AuvCameraServoRemote.h"
#include "AuvCameraServoConfig.h"
#include "AuvProtocol.h"

static AuvCameraServoConfig CalibratedConfig(void)
{
    AuvCameraServoConfig config = {1U, 2250U, 3000U, 3000U, 100U};
    return config;
}

static void BuildCommand(uint8_t payload[9], uint32_t sequence, float position)
{
    AuvProtocol_WriteU32Le(payload, sequence);
    payload[4] = AUV_CAMERA_SERVO_ACTUATOR_ID;
    AuvProtocol_WriteF32Le(&payload[5], position);
}

static void TestUncalibratedHoldsStartup(void)
{
    AuvCameraServoConfig config = CalibratedConfig();
    uint8_t payload[9];
    config.calibrated = 0U;
    AuvCameraServo_Init(&config);
    BuildCommand(payload, 1U, -1.0f);
    assert(AuvCameraServo_Accept(payload, sizeof(payload), 1U, 1U) ==
           AUV_ARM_UNSAFE);
    AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 3000U);
    assert(AuvCameraServo_GetStatus()->state == AUV_CAMERA_SERVO_UNCALIBRATED);
}

static void TestProtocolBoundsSlewAndAuthorization(void)
{
    AuvCameraServoConfig config = CalibratedConfig();
    uint8_t payload[9];
    AuvCameraServo_Init(&config);

    BuildCommand(payload, 10U, -1.0f);
    assert(AuvCameraServo_Accept(payload, sizeof(payload), 0U, 1U) ==
           AUV_ARM_DISARMED);
    assert(AuvCameraServo_Accept(payload, sizeof(payload), 1U, 0U) ==
           AUV_ARM_UNSAFE);
    assert(AuvCameraServo_Accept(payload, sizeof(payload), 1U, 1U) ==
           AUV_ARM_ACCEPTED);
    AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 2900U);
    AuvCameraServo_Tick(0U);
    assert(AuvCameraServo_GetCcr() == 2900U);

    BuildCommand(payload, 11U, 1.0f);
    assert(AuvCameraServo_Accept(payload, sizeof(payload), 1U, 1U) ==
           AUV_ARM_ACCEPTED);
    assert(AuvCameraServo_GetStatus()->target_ccr == 3000U);
    assert(AuvCameraServo_Accept(payload, sizeof(payload), 1U, 1U) ==
           AUV_ARM_MALFORMED);

    BuildCommand(payload, 12U, 0.0f);
    assert(AuvCameraServo_Accept(payload, sizeof(payload), 1U, 1U) ==
           AUV_ARM_ACCEPTED);
    assert(AuvCameraServo_GetStatus()->target_ccr == 2625U);
}

static void TestRemoteMapping(void)
{
    AuvCameraServoConfig config = CalibratedConfig();
    AuvCameraServo_Init(&config);
    AuvCameraServo_CommandRemote(0U, 1U, 1U);
    assert(AuvCameraServo_GetStatus()->target_ccr == 2250U);
    AuvCameraServo_CommandRemote(255U, 1U, 1U);
    assert(AuvCameraServo_GetStatus()->target_ccr == 3000U);
}

static void TestInstalledLimitsAndSelection(void)
{
    AuvCameraServoConfig config = {1U, AUV_CAMERA_SERVO_MIN_CCR,
        AUV_CAMERA_SERVO_MAX_CCR, AUV_CAMERA_SERVO_STARTUP_CCR, 10U};
    AuvCameraServo_Init(&config);
    assert(AuvCameraServo_GetCcr() == 3000U); /* zero, not range midpoint */
    AuvCameraServo_SelectRemote(1U, 255U, 0U, 1U); /* unverified SI */
    AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 3000U);
    AuvCameraServo_SelectRemote(1U, 0U, 1U, 1U);
    assert(AuvCameraServo_GetStatus()->target_ccr == 2550U);
    AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 2990U);
    AuvCameraServo_SelectRemote(0U, 255U, 1U, 1U); /* down cancels ramp */
    AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 2990U);
    AuvCameraServo_SelectRemote(2U, 255U, 1U, 1U); /* main placeholder */
    AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 2990U);
    AuvCameraServo_SelectRemote(1U, 255U, 1U, 1U);
    for(int i=0;i<100;i++) AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 3100U);
    AuvCameraServo_SelectRemote(1U, 0U, 1U, 1U);
    for(int i=0;i<100;i++) AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 2550U);
    AuvCameraServo_SelectRemote(1U, 255U, 1U, 0U);
    AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 2550U);
    AuvCameraServo_SelectRemote(3U, 255U, 1U, 1U);
    AuvCameraServo_Tick(1U);
    assert(AuvCameraServo_GetCcr() == 2550U);
}

int main(void)
{
    TestUncalibratedHoldsStartup();
    TestProtocolBoundsSlewAndAuthorization();
    TestRemoteMapping();
    TestInstalledLimitsAndSelection();
    puts("test_auv_camera_servo: PASS");
    return 0;
}
