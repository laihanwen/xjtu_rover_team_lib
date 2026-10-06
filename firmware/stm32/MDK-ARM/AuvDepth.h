/**
 * @file AuvDepth.h
 * @brief Hardware-independent depth sample cache and freshness gate.
 */

#ifndef AUV_DEPTH_H
#define AUV_DEPTH_H

#include <stdint.h>

/* M10 documented output is 0.5..1 Hz: allow one 2 s period plus margin. */
#define AUV_DEPTH_TIMEOUT_MS 3000U
#define AUV_DEPTH_MAX_METERS 20.0f

typedef struct {
    float depth_m;
    uint32_t sample_sequence;
    uint32_t last_update_ms;
    uint8_t valid;
} AuvDepthSample;

void AuvDepth_Init(void);
uint8_t AuvDepth_Update(float depth_m, uint32_t now_ms);
/* Signed M10 gauge reading; only relative manual hold uses this datum. */
uint8_t AuvDepth_UpdateGauge(float depth_m, uint32_t now_ms);
void AuvDepth_Invalidate(uint32_t now_ms);
uint8_t AuvDepth_CopyFresh(uint32_t now_ms, AuvDepthSample *sample);

#endif
