#ifndef AUV_ROV_DEPTH_H
#define AUV_ROV_DEPTH_H
#include <stdint.h>
#include "AuvDepth.h"
typedef struct {
    char line[96];
    uint8_t length;
    uint8_t overflow;
} AuvM10Parser;
void AuvM10_Reset(AuvM10Parser *parser);
/* 0=incomplete, 1=valid line, -1=invalid line; never publishes stale samples. */
int AuvM10_Push(AuvM10Parser *parser, uint8_t byte, float *depth);
typedef struct {
    float integral;
    float target;
    uint32_t sequence;
    uint32_t sample_ms;
    float output;
    uint8_t active;
} AuvRovDepthControl;
void AuvRovDepth_Reset(AuvRovDepthControl *control);
float AuvRovDepth_Step(AuvRovDepthControl *control, const AuvDepthSample *sample,
                      uint8_t enabled, uint8_t lock_current, float target);
#endif
