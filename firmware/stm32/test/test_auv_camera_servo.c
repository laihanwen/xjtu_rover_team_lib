#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "AuvCameraServo.h"
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

int main(void)
{
    TestUncalibratedHoldsStartup();
    TestProtocolBoundsSlewAndAuthorization();
    TestRemoteMapping();
    puts("test_auv_camera_servo: PASS");
    return 0;
}
