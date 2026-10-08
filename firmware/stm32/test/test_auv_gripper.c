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

static void TestBootHoldsAndSmallSelfTest(void)
{
    AuvGripperConfig config = CalibratedConfig();
    AuvGripperBootTestConfig boot = {1U, 1500U, 1600U, 100U, 50U};
    AuvGripperStatus status;
    uint8_t payload[9];
    AuvGripper_Init(&config);
    for (int i=0;i<10;++i) AuvGripper_Tick();
    assert(AuvGripper_GetPulseUs()==1500U);
    assert(AuvGripper_ConfigureBootTest(&boot, 0xfffffff0U));
    AuvGripper_BootTestTick(0x53U); /* 99ms across rollover */
    AuvGripper_Tick();
    assert(AuvGripper_GetPulseUs()==1500U);
    AuvGripper_BootTestTick(0x54U);
    BuildCommand(payload,1U,1.0f);
    assert(AuvGripper_Accept(payload,9U,1U,1U)==AUV_ARM_UNSAFE);
    AuvGripper_Tick();
    assert(AuvGripper_GetPulseUs()==1600U);
    AuvGripper_BootTestTick(0x55U);
    AuvGripper_BootTestTick(0x86U);
    AuvGripper_Tick();
    assert(AuvGripper_GetPulseUs()==1600U);
    AuvGripper_BootTestTick(0x87U);
    AuvGripper_Tick();
    AuvGripper_BootTestTick(0x88U);
    AuvGripper_GetStatus(&status);
    assert(status.boot_test_state==AUV_GRIPPER_TEST_COMPLETE);
    assert(status.current_us==1500U);
    assert(!AuvGripper_ConfigureBootTest(&boot,1000U));
    AuvGripper_BootTestTick(10000U);
    assert(AuvGripper_GetPulseUs()==1500U);
}

static void TestBootTestCancelAndInvalid(void)
{
    AuvGripperConfig config=CalibratedConfig();
    AuvGripperBootTestConfig boot={1U,1500U,1600U,0U,0U};
    AuvGripperStatus status;
    AuvGripper_Init(&config);
    assert(AuvGripper_ConfigureBootTest(&boot,0U));
    AuvGripper_BootTestTick(0U);
    AuvGripper_CancelBootTest();
    AuvGripper_Tick();
    AuvGripper_GetStatus(&status);
    assert(status.boot_test_state==AUV_GRIPPER_TEST_CANCELLED);
    assert(status.current_us==1500U);
    AuvGripper_Init(&config);
    boot.excursion_us=2100U; /* inside PWM range, outside calibrated mechanism */
    assert(!AuvGripper_ConfigureBootTest(&boot,0U));
    assert(!AuvGripper_PwmEnabled());
    {
        uint8_t stop[9];
        BuildCommand(stop, 2U, 0.0f);
        assert(AuvGripper_Accept(stop, 9U, 0U, 0U)==AUV_ARM_ACCEPTED);
        assert(!AuvGripper_PwmEnabled());
    }
    config.calibrated=0U;
    AuvGripper_Init(&config);
    assert(!AuvGripper_PwmEnabled());
    boot.enabled=0U;
    assert(AuvGripper_ConfigureBootTest(&boot,0U));
    assert(!AuvGripper_PwmEnabled());
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
    assert(AuvGripper_GetPulseUs() == 1600U);
    assert(AuvGripper_Accept(payload, sizeof(payload), 1U, 1U) == AUV_ARM_MALFORMED);

    BuildCommand(payload, 13U, 0.0f);
    assert(AuvGripper_Accept(payload, sizeof(payload), 0U, 0U) == AUV_ARM_ACCEPTED);
    AuvGripper_GetStatus(&status);
    assert(status.state == AUV_GRIPPER_STOPPED);
    assert(status.target_us == 1600U);
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
    TestBootHoldsAndSmallSelfTest();
    TestBootTestCancelAndInvalid();
    TestProtocolOpenStopAndSequence();
    TestRemoteThresholds();
    puts("test_auv_gripper: PASS");
    return 0;
}
