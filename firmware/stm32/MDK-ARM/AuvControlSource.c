#include "AuvControlSource.h"
#include "AuvMode.h"
#if AUV_DUAL_PROFILE
static volatile uint8_t operating_mode;
static uint8_t mode_sequence_seen;
static uint32_t mode_sequence, mode_changed_ms;
uint8_t AuvMode_Get(void) { return operating_mode; }
uint8_t AuvMode_ArmReady(uint32_t now_ms) { return (uint32_t)(now_ms-mode_changed_ms)>=200U; }
AuvArmResult AuvMode_Select(uint32_t sequence, uint8_t mode, uint8_t armed, uint32_t now_ms)
{
    if(mode>AUV_MODE_AUV) return AUV_ARM_MALFORMED;
    if(armed) return AUV_ARM_UNSAFE;
    if(mode_sequence_seen && (int32_t)(sequence-mode_sequence)<=0) return AUV_ARM_MALFORMED;
    mode_sequence_seen=1U;mode_sequence=sequence;mode_changed_ms=now_ms;
    operating_mode=mode;
    (void)AuvControlSource_Update(0U,0U,0U);
    return AUV_ARM_ACCEPTED;
}
#endif

static AuvControlSource active_source = AUV_CONTROL_SOURCE_NONE;

void AuvControlSource_Init(void)
{
    active_source = AUV_CONTROL_SOURCE_NONE;
#if AUV_DUAL_PROFILE
    operating_mode=AUV_MODE_ROV;mode_sequence_seen=0U;mode_sequence=mode_changed_ms=0U;
#endif
}

AuvControlSourceDecision AuvControlSource_Update(uint8_t armed,
                                                 uint8_t pi_fresh,
                                                 uint8_t rc_fresh)
{
    AuvControlSourceDecision decision;
    decision.disarm_required = 0U;
#if AUV_DUAL_PROFILE
    if(AUV_MODE_IS_ROV) pi_fresh=0U;
    else rc_fresh=0U;
#endif

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

AuvControlSource AuvControlSource_GetActive(void)
{
    return active_source;
}
