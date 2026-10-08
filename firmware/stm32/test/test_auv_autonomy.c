#include "AuvMotionTarget.h"
#include "AuvProtocol.h"
#include "AuvRovConfig.h"
#include <assert.h>
#include <math.h>

static AuvArmResult accept(float vx, float depth) {
    uint8_t payload[20] = {0};
    AuvMotionTarget_Init();
    AuvProtocol_WriteU32Le(payload, 1U);
    AuvProtocol_WriteF32Le(payload + 4, vx);
    AuvProtocol_WriteF32Le(payload + 12, depth);
    return AuvMotionTarget_Accept(payload, 20U, 1U, 0U);
}
int main(void) {
    assert(AUV_AUTONOMOUS_PROFILE && !AUV_ROV_MANUAL_TRIAL);
    assert(!AUV_AUTONOMY_READY); /* Selecting AUV cannot commission hardware. */
    assert(accept(0.2f, 0.0f) == AUV_ARM_DISARMED);
    assert(accept(0.201f, 0.0f) == AUV_ARM_MALFORMED);
    assert(accept(-0.201f, 0.0f) == AUV_ARM_MALFORMED);
    assert(accept(0.0f, 0.01f) == AUV_ARM_MALFORMED); /* Unknown depth limit. */
    assert(accept(NAN, 0.0f) == AUV_ARM_MALFORMED);
    return 0;
}
