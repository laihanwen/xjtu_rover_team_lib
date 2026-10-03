#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "AuvGripper.h"
#include "AuvGripperConfig.h"
#include "AuvProtocol.h"

static AuvGripperConfig CalibratedConfig(void)
{
    AuvGripperConfig config = {1U, 500U, 1500U, 2500U, 1000U, 2000U, 100U};
    return config;
}

static void BuildCommand(uint8_t payload[9], uint32_t sequence, float command)
{
    AuvProtocol_WriteU32Le(&payload[0], sequence);
    payload[4] = AUV_GRIPPER_ACTUATOR_ID;
    AuvProtocol_WriteF32Le(&payload[5], command);
}

static void TestUncalibratedRejectsMotion(void)
{
    AuvGripperConfig config = CalibratedConfig();
    AuvGripperStatus status;
    uint8_t payload[9];
    config.calibrated = 0U;
    AuvGripper_Init(&config);
    BuildCommand(payload, 1U, 1.0f);
    assert(AuvGripper_Accept(payload, sizeof(payload), 1U, 1U) == AUV_ARM_UNSAFE);
    AuvGripper_GetStatus(&status);
    assert(status.state == AUV_GRIPPER_UNCALIBRATED);
    assert(status.current_us == 1500U);
    assert((status.error_flags & AUV_GRIPPER_ERROR_UNCALIBRATED) != 0U);
}

static void TestBootClosesWithSlew(void)
{
    AuvGripperConfig config = CalibratedConfig();
    AuvGripperStatus status;
    AuvGripper_Init(&config);
    AuvGripper_GetStatus(&status);
    assert(status.state == AUV_GRIPPER_MOVING_CLOSE);
    assert(status.target_us == 1000U);
    for (int i = 0; i < 5; ++i) AuvGripper_Tick();
    AuvGripper_GetStatus(&status);
    assert(status.current_us == 1000U);
    assert(status.state == AUV_GRIPPER_CLOSED);
}

static void TestProtocolOpenStopAndSequence(void)
{
    AuvGripperConfig config = CalibratedConfig();
    AuvGripperStatus status;
    uint8_t payload[9];
    AuvGripper_Init(&config);
    for (int i = 0; i < 5; ++i) AuvGripper_Tick();

    BuildCommand(payload, 10U, 1.0f);
    assert(AuvGripper_Accept(payload, sizeof(payload), 0U, 1U) == AUV_ARM_DISARMED);
    BuildCommand(payload, 11U, 1.0f);
    assert(AuvGripper_Accept(payload, sizeof(payload), 1U, 0U) == AUV_ARM_UNSAFE);
    BuildCommand(payload, 12U, 1.0f);
    assert(AuvGripper_Accept(payload, sizeof(payload), 1U, 1U) == AUV_ARM_ACCEPTED);
    AuvGripper_Tick();
    assert(AuvGripper_GetPulseUs() == 1100U);
    assert(AuvGripper_Accept(payload, sizeof(payload), 1U, 1U) == AUV_ARM_MALFORMED);

    BuildCommand(payload, 13U, 0.0f);
    assert(AuvGripper_Accept(payload, sizeof(payload), 0U, 0U) == AUV_ARM_ACCEPTED);
    AuvGripper_GetStatus(&status);
    assert(status.state == AUV_GRIPPER_STOPPED);
    assert(status.target_us == 1100U);
}

static void TestRemoteThresholds(void)
{
    AuvGripperConfig config = CalibratedConfig();
    AuvGripperStatus status;
    AuvGripper_Init(&config);
    for (int i = 0; i < 5; ++i) AuvGripper_Tick();
    AuvGripper_CommandRemote(200U, 1U, 1U);
    AuvGripper_GetStatus(&status);
    assert(status.state == AUV_GRIPPER_MOVING_OPEN);
    AuvGripper_CommandRemote(127U, 1U, 1U);
    AuvGripper_GetStatus(&status);
    assert(status.state == AUV_GRIPPER_STOPPED);
}

int main(void)
{
    /* Safe even if the PWM task starts before AuvGripper_Init(). */
    assert(AuvGripper_GetPulseUs() == AUV_GRIPPER_NEUTRAL_US);

    TestUncalibratedRejectsMotion();
    TestBootClosesWithSlew();
    TestProtocolOpenStopAndSequence();
    TestRemoteThresholds();
    puts("test_auv_gripper: PASS");
    return 0;
}
