#include "AuvMotionTarget.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "AuvProtocol.h"

static void MakePayload(uint8_t payload[20], uint32_t sequence,
                        float vx, float vy, float depth, float yaw)
{
    AuvProtocol_WriteU32Le(&payload[0], sequence);
    AuvProtocol_WriteF32Le(&payload[4], vx);
    AuvProtocol_WriteF32Le(&payload[8], vy);
    AuvProtocol_WriteF32Le(&payload[12], depth);
    AuvProtocol_WriteF32Le(&payload[16], yaw);
}

int main(void)
{
    uint8_t payload[20];
    AuvMotionTarget target;

    AuvMotionTarget_Init();
    MakePayload(payload, 1U, 0.5f, -0.25f, 2.0f, 1.0f);
    assert(AuvMotionTarget_Accept(payload, sizeof(payload), 10U, 0U) ==
           AUV_ARM_DISARMED);
    assert(AuvMotionTarget_CopyFresh(10U, &target) == 1U);
    MakePayload(payload, 2U, 0.5f, -0.25f, 2.0f, 1.0f);
    assert(AuvMotionTarget_Accept(payload, sizeof(payload), 10U, 1U) ==
           AUV_ARM_ACCEPTED);
    assert(AuvMotionTarget_CopyFresh(10U, &target) == 1U);
    assert(target.sequence == 2U && target.vx == 0.5f && target.vy == -0.25f);
    assert(AuvMotionTarget_Accept(payload, sizeof(payload), 11U, 1U) ==
           AUV_ARM_MALFORMED);
    assert(AuvMotionTarget_CopyFresh(10U + AUV_MOTION_TARGET_TIMEOUT_MS,
                                     &target) == 1U);
    assert(AuvMotionTarget_CopyFresh(11U + AUV_MOTION_TARGET_TIMEOUT_MS,
                                     &target) == 0U);

    MakePayload(payload, 3U, NAN, 0.0f, 0.0f, 0.0f);
    assert(AuvMotionTarget_Accept(payload, sizeof(payload), 20U, 1U) ==
           AUV_ARM_MALFORMED);
    MakePayload(payload, 3U, 0.0f, 0.0f, 0.0f, 4.0f);
    assert(AuvMotionTarget_Accept(payload, sizeof(payload), 20U, 1U) ==
           AUV_ARM_MALFORMED);
    return 0;
}
