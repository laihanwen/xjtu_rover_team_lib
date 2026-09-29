#include "AuvControlSource.h"

static AuvControlSource active_source = AUV_CONTROL_SOURCE_NONE;

void AuvControlSource_Init(void)
{
    active_source = AUV_CONTROL_SOURCE_NONE;
}

AuvControlSourceDecision AuvControlSource_Update(uint8_t armed,
                                                 uint8_t pi_fresh,
                                                 uint8_t rc_fresh)
{
    AuvControlSourceDecision decision;
    decision.disarm_required = 0U;

    if (armed == 0U) {
        active_source = AUV_CONTROL_SOURCE_NONE;
    } else if (active_source == AUV_CONTROL_SOURCE_NONE) {
        if (pi_fresh != 0U)
            active_source = AUV_CONTROL_SOURCE_PI;
        else if (rc_fresh != 0U)
            active_source = AUV_CONTROL_SOURCE_RC;
        else
            decision.disarm_required = 1U;
    } else if (((active_source == AUV_CONTROL_SOURCE_PI) &&
                (pi_fresh == 0U)) ||
               ((active_source == AUV_CONTROL_SOURCE_RC) &&
                (rc_fresh == 0U))) {
        active_source = AUV_CONTROL_SOURCE_NONE;
        decision.disarm_required = 1U;
    }

    decision.active = active_source;
    return decision;
}
