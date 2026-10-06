/**
 * @file AuvDepth.c
 * @brief Hardware-independent depth sample cache and freshness gate.
 */

#include "AuvDepth.h"

#include <stddef.h>

static volatile uint32_t generation;
static volatile float active_depth_m;
static volatile uint32_t active_sequence;
static volatile uint32_t active_update_ms;
static volatile uint8_t active_valid;

void AuvDepth_Init(void)
{
    generation = 0U;
    active_depth_m = 0.0f;
    active_sequence = 0U;
    active_update_ms = 0U;
    active_valid = 0U;
}

uint8_t AuvDepth_Update(float depth_m, uint32_t now_ms)
{
    if (depth_m != depth_m || depth_m < 0.0f || depth_m > AUV_DEPTH_MAX_METERS)
        return 0U;
    generation++;
    active_depth_m = depth_m;
    active_update_ms = now_ms;
    active_sequence++;
    active_valid = 1U;
    generation++;
    return 1U;
}

void AuvDepth_Invalidate(uint32_t now_ms)
{
    generation++;
    active_update_ms = now_ms;
    active_sequence++;
    active_valid = 0U;
    generation++;
}

uint8_t AuvDepth_CopyFresh(uint32_t now_ms, AuvDepthSample *sample)
{
    uint32_t before;
    uint32_t after;
    AuvDepthSample snapshot;
    if (sample == NULL) return 0U;
    for (;;) {
        before = generation;
        if ((before & 1U) != 0U) continue;
        snapshot.depth_m = active_depth_m;
        snapshot.sample_sequence = active_sequence;
        snapshot.last_update_ms = active_update_ms;
        snapshot.valid = active_valid;
        after = generation;
        if (before == after && (after & 1U) == 0U) break;
    }
    *sample = snapshot;
    return (snapshot.valid != 0U &&
            (uint32_t)(now_ms - snapshot.last_update_ms) <= AUV_DEPTH_TIMEOUT_MS) ? 1U : 0U;
}
