#include "AuvControlSource.h"

#include <assert.h>

int main(void)
{
    AuvControlSourceDecision decision;

    AuvControlSource_Init();
    decision = AuvControlSource_Update(0U, 1U, 1U);
    assert(decision.active == AUV_CONTROL_SOURCE_NONE);
    assert(decision.disarm_required == 0U);
    assert(AuvControlSource_GetActive() == AUV_CONTROL_SOURCE_NONE);

    decision = AuvControlSource_Update(1U, 1U, 1U);
    assert(decision.active == AUV_CONTROL_SOURCE_PI);
    assert(decision.disarm_required == 0U);
    assert(AuvControlSource_GetActive() == AUV_CONTROL_SOURCE_PI);

    decision = AuvControlSource_Update(1U, 1U, 0U);
    assert(decision.active == AUV_CONTROL_SOURCE_PI);
    assert(decision.disarm_required == 0U);

    decision = AuvControlSource_Update(1U, 0U, 1U);
    assert(decision.active == AUV_CONTROL_SOURCE_NONE);
    assert(decision.disarm_required == 1U);

    /* It may select RC only after the explicit disarm/re-arm boundary. */
    decision = AuvControlSource_Update(0U, 0U, 1U);
    assert(decision.active == AUV_CONTROL_SOURCE_NONE);
    decision = AuvControlSource_Update(1U, 0U, 1U);
    assert(decision.active == AUV_CONTROL_SOURCE_RC);

    decision = AuvControlSource_Update(1U, 1U, 0U);
    assert(decision.active == AUV_CONTROL_SOURCE_NONE);
    assert(decision.disarm_required == 1U);

    AuvControlSource_Init();
    decision = AuvControlSource_Update(1U, 0U, 0U);
    assert(decision.active == AUV_CONTROL_SOURCE_NONE);
    assert(decision.disarm_required == 1U);
    return 0;
}
