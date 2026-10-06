#include "AuvRcInput.h"

#include <assert.h>
#include <stdint.h>

static void PushFrame(const uint8_t frame[AUV_RC_FRAME_SIZE], uint32_t now_ms)
{
    uint32_t i;
    for (i = 0U; i < AUV_RC_FRAME_SIZE; ++i)
        AuvRcInput_PushByte(frame[i], now_ms);
}

int main(void)
{
    const uint8_t expected[AUV_RC_FRAME_SIZE] =
        {0xA5U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U};
    uint8_t actual[AUV_RC_FRAME_SIZE] = {0U};
    uint32_t i;

    AuvRcInput_Init();
    assert(AuvRcInput_CopyFreshFrame(0U, actual) == 0U);

    AuvRcInput_PushByte(0x44U, 10U);
    PushFrame(expected, 20U);
    assert(AuvRcInput_CopyFreshFrame(20U, actual) == 1U);
    for (i = 0U; i < AUV_RC_FRAME_SIZE; ++i) assert(actual[i] == expected[i]);

    assert(AuvRcInput_CopyFreshFrame(20U + AUV_RC_TIMEOUT_MS, actual) == 1U);
    assert(AuvRcInput_CopyFreshFrame(21U + AUV_RC_TIMEOUT_MS, actual) == 0U);

    /* Unsigned subtraction preserves timeout behavior across tick wraparound. */
    AuvRcInput_Init();
    PushFrame(expected, UINT32_MAX - 5U);
    assert(AuvRcInput_CopyFreshFrame(4U, actual) == 1U);
    AuvRcInput_Init();
    {
        uint8_t frame[11]={0xa5,127,127,127,127,127,0,2,0,0,0};
        assert(AuvRcInput_AcceptCrc(1U,frame,1U,100U));
        assert(AuvRcInput_CanArm(100U));
        assert(!AuvRcInput_AcceptCrc(1U,frame,1U,101U));
        assert(!AuvRcInput_CanArm(351U));
        frame[1]=200U;
        assert(AuvRcInput_AcceptCrc(2U,frame,1U,352U));
        assert(!AuvRcInput_CanArm(352U));
        frame[1]=127U;
        assert(AuvRcInput_AcceptCrc(3U,frame,0U,353U));
        assert(!AuvRcInput_CanArm(353U));
        frame[7]=1U;
        assert(!AuvRcInput_AcceptCrc(4U,frame,1U,354U));
    }
    return 0;
}
