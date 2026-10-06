#ifndef AUV_ANGLES_H
#define AUV_ANGLES_H
#include <math.h>

static inline float AuvAngles_Wrap180(float angle)
{
    if (!isfinite(angle)) return angle;
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
#endif
