#ifndef AUV_H30_FRAME_H
#define AUV_H30_FRAME_H
#include <stdint.h>
#include <string.h>
typedef struct {
    uint8_t bytes[262];
    uint16_t count;
    uint32_t last_byte_ms;
    uint32_t checksum_errors;
    uint32_t timeouts;
} AuvH30Frame;

/* Preserve candidate headers after corruption instead of discarding the next frame. */
static inline uint8_t AuvH30Frame_Push(AuvH30Frame *p, uint8_t byte, uint32_t now)
{
    if (p->count && (uint32_t)(now - p->last_byte_ms) > 20U) {
        p->count = 0U;
        p->timeouts++;
    }
    p->last_byte_ms = now;
    if (p->count == sizeof(p->bytes)) p->count = 0U;
    p->bytes[p->count++] = byte;
    for (;;) {
        uint16_t length, i;
        uint8_t a = 0U, b = 0U;
        if (p->bytes[0] != 0x59U ||
            (p->count > 1U && p->bytes[1] != 0x53U)) {
            if (--p->count == 0U) return 0U;
            memmove(p->bytes, p->bytes + 1, p->count);
            continue;
        }
        if (p->count < 5U) return 0U;
        length = (uint16_t)(p->bytes[4] + 7U);
        if (p->count < length) return 0U;
        for (i = 2U; i < length - 2U; ++i) {
            a = (uint8_t)(a + p->bytes[i]);
            b = (uint8_t)(b + a);
        }
        if (a == p->bytes[length - 2U] && b == p->bytes[length - 1U])
            return 1U;
        p->checksum_errors++;
        --p->count;
        memmove(p->bytes, p->bytes + 1, p->count);
    }
}
#endif
