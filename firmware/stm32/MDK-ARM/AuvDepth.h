/**
 * @file AuvDepth.h
 * @brief Hardware-independent depth sample cache and freshness gate.
 */

#ifndef AUV_DEPTH_H
#define AUV_DEPTH_H

#include <stdint.h>

#define AUV_DEPTH_TIMEOUT_MS 250U
#define AUV_DEPTH_MAX_METERS 20.0f

typedef struct {
    float depth_m;
    uint32_t sample_sequence;
    uint32_t last_update_ms;
    uint8_t valid;
} AuvDepthSample;

void AuvDepth_Init(void);
uint8_t AuvDepth_Update(float depth_m, uint32_t now_ms);
void AuvDepth_Invalidate(uint32_t now_ms);
uint8_t AuvDepth_CopyFresh(uint32_t now_ms, AuvDepthSample *sample);

#endif
