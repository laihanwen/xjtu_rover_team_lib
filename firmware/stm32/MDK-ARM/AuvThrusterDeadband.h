#ifndef AUV_THRUSTER_DEADBAND_H
#define AUV_THRUSTER_DEADBAND_H
#include <math.h>
#include <stdint.h>
/* Preserve neutral and sign, mapping active demands into measured usable span. */
static inline float AuvThruster_Compensate(float demand, float limit, float start)
{
    float magnitude = fabsf(demand);
    if (!isfinite(demand) || limit <= start || start < 0 || magnitude < 1.0f) return 0;
    if (magnitude > limit) magnitude = limit;
    magnitude = start + (limit-start)*magnitude/limit;
    return demand < 0 ? -magnitude : magnitude;
}
static inline uint8_t AuvAttitude_Active(float angle, uint8_t active)
{
    if (!isfinite(angle)) return 0;
    return fabsf(angle) >= (active ? 1.0f : 2.0f);
}
static inline float AuvThruster_Slew(float current, float target, float step)
{
    if (!isfinite(current) || !isfinite(target) || !isfinite(step) || step < 0) return 0;
    if (target-current > step) return current+step;
    if (current-target > step) return current-step;
    return target;
}
#endif
