#ifndef AUV_SAMPLE_AGE_H
#define AUV_SAMPLE_AGE_H
#include <stdint.h>
/* Call with a coherent snapshot: sample timestamp must be read before now. */
static inline uint8_t AuvSample_IsFresh(uint32_t seen,uint32_t sample_ms,
                                      uint32_t now_ms,uint32_t timeout_ms)
{
    return seen && (uint32_t)(now_ms-sample_ms)<=timeout_ms;
}
#endif
