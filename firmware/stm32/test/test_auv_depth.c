#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "AuvDepth.h"

int main(void)
{
    AuvDepthSample sample;
    AuvDepth_Init();
    assert(AuvDepth_CopyFresh(0U, &sample) == 0U);
    assert(AuvDepth_Update(NAN, 10U) == 0U);
    assert(AuvDepth_Update(-0.1f, 10U) == 0U);
    assert(AuvDepth_Update(AUV_DEPTH_MAX_METERS + 0.1f, 10U) == 0U);
    assert(AuvDepth_Update(1.25f, 10U) != 0U);
    assert(AuvDepth_CopyFresh(10U+AUV_DEPTH_TIMEOUT_MS, &sample) != 0U);
    assert(sample.depth_m == 1.25f);
    assert(sample.sample_sequence == 1U);
    assert(AuvDepth_CopyFresh(11U+AUV_DEPTH_TIMEOUT_MS, &sample) == 0U);
    AuvDepth_Invalidate(262U);
    assert(AuvDepth_CopyFresh(262U, &sample) == 0U);
    assert(AuvDepth_UpdateGauge(-0.22f, 300U));
    assert(AuvDepth_CopyFresh(2300U,&sample));
    assert(sample.depth_m == -0.22f);
    assert(!AuvDepth_UpdateGauge(-21.0f,300U));

    AuvDepth_Init();
    assert(AuvDepth_Update(2.0f, 0xFFFFFFF0U) != 0U);
    assert(AuvDepth_CopyFresh(0x00000010U, &sample) != 0U);
    puts("test_auv_depth: PASS");
    return 0;
}
