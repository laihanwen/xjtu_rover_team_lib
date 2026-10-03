#ifndef AUV_CONTROL_SOURCE_H
#define AUV_CONTROL_SOURCE_H

#include <stdint.h>

typedef enum {
    AUV_CONTROL_SOURCE_NONE = 0,
    AUV_CONTROL_SOURCE_RC,
    AUV_CONTROL_SOURCE_PI
} AuvControlSource;

typedef struct {
    AuvControlSource active;
    uint8_t disarm_required;
} AuvControlSourceDecision;

void AuvControlSource_Init(void);
AuvControlSourceDecision AuvControlSource_Update(uint8_t armed,
                                                 uint8_t pi_fresh,
                                                 uint8_t rc_fresh);
AuvControlSource AuvControlSource_GetActive(void);

#endif
