#include "AuvMode.h"
#include "AuvControlSource.h"
#include "AuvMotionTarget.h"
#include "AuvProtocol.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    uint8_t target[20]={0};
    AuvControlSource_Init();AuvMotionTarget_Init();
    assert(AUV_MODE_IS_ROV && !AUV_MODE_IS_AUV);
    assert(AuvControlSource_Update(1,1,1).active==AUV_CONTROL_SOURCE_RC);
    assert(AuvMode_Select(1,1,1,100)==AUV_ARM_UNSAFE);
    assert(AUV_MODE_IS_ROV);
    assert(AuvMode_Select(1,2,0,100)==AUV_ARM_MALFORMED);
    assert(AuvMode_Select(1,1,0,100)==AUV_ARM_ACCEPTED);
    assert(AUV_MODE_IS_AUV && !AuvMode_ArmReady(299) && AuvMode_ArmReady(300));
    assert(AuvControlSource_GetActive()==AUV_CONTROL_SOURCE_NONE);
    assert(AuvControlSource_Update(1,1,1).active==AUV_CONTROL_SOURCE_PI);
    assert(!AUV_AUTONOMY_READY); /* Selecting mode never commissions hardware. */
    assert(AuvMode_Select(1,0,0,400)==AUV_ARM_MALFORMED);
    AuvProtocol_WriteU32Le(target,1);
    AuvProtocol_WriteF32Le(target+4,.21f);
    assert(AuvMotionTarget_Accept(target,20,400,0)==AUV_ARM_MALFORMED);
    AuvProtocol_WriteF32Le(target+4,.08f);
    AuvProtocol_WriteF32Le(target+12,1.21f);
    assert(AuvMotionTarget_Accept(target,20,400,0)==AUV_ARM_MALFORMED);
    AuvProtocol_WriteF32Le(target+12,.7f);
    assert(AuvMotionTarget_Accept(target,20,400,0)==AUV_ARM_DISARMED);
    assert(AuvMode_Select(2,0,0,500)==AUV_ARM_ACCEPTED && AUV_MODE_IS_ROV);
    AuvControlSource_Init();
    assert(AUV_MODE_IS_ROV); /* Reboot is predictable and never resumes AUV. */
    puts("dual-mode interlock passed");return 0;
}
