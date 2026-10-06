#include "AuvAngles.h"
#include <assert.h>
int main(void)
{
    /* Crossing the IMU +/-180 boundary must not create a 358-degree error. */
    assert(AuvAngles_Wrap180(-179.0f - 179.0f) == 2.0f);
    assert(AuvAngles_Wrap180(179.0f - -179.0f) == -2.0f);
    assert(AuvAngles_Wrap180(180.0f) == -180.0f);
    assert(AuvAngles_Wrap180(-180.0f) == -180.0f);
    assert(AuvAngles_Wrap180(721.0f) == 1.0f);
    assert(AuvAngles_Wrap180(-721.0f) == -1.0f);
    assert(isnan(AuvAngles_Wrap180(NAN)));
    assert(isinf(AuvAngles_Wrap180(INFINITY)));
    return 0;
}
