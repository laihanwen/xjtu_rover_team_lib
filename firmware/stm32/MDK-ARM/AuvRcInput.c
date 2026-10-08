/**
 * @file AuvRcInput.c
 * @brief Stream framing and timeout handling for legacy remote-control data.
 */

#include "AuvRcInput.h"

#include <stddef.h>

static uint8_t assembling[AUV_RC_FRAME_SIZE];
static uint8_t published[AUV_RC_FRAME_SIZE];
static uint8_t assembling_length;
static volatile uint32_t publish_sequence;
static volatile uint32_t last_frame_ms;
static volatile uint8_t frame_seen;
static uint8_t crc_seen;
static uint32_t crc_sequence;
static volatile uint8_t crc_deadman;

void AuvRcInput_Init(void) {
  uint32_t i;
  assembling_length = 0U;
  publish_sequence = 0U;
  last_frame_ms = 0U;
  frame_seen = 0U;
  crc_seen = 0U;
  crc_deadman = 0U;
  crc_sequence = 0U;
  for (i = 0U; i < AUV_RC_FRAME_SIZE; ++i) {
    assembling[i] = 0U;
    published[i] = 0U;
  }
}

void AuvRcInput_PushByte(uint8_t byte, uint32_t now_ms) {
  uint32_t i;

  if (assembling_length == 0U) {
    if (byte != AUV_RC_FRAME_HEADER)
      return;
    assembling[0] = byte;
    assembling_length = 1U;
    return;
  }

  assembling[assembling_length++] = byte;
  if (assembling_length < AUV_RC_FRAME_SIZE)
    return;

  /* Odd sequence means that the interrupt is publishing a new snapshot. */
  ++publish_sequence;
  for (i = 0U; i < AUV_RC_FRAME_SIZE; ++i)
    published[i] = assembling[i];
  last_frame_ms = now_ms;
  frame_seen = 1U;
  ++publish_sequence;
  assembling_length = 0U;
}

uint8_t AuvRcInput_CopyFreshFrame(uint32_t now_ms,
                                  uint8_t frame[AUV_RC_FRAME_SIZE]) {
  uint32_t before;
  uint32_t after;
  uint32_t timestamp;
  uint32_t i;
  uint8_t seen;

  if (frame == NULL)
    return 0U;
  for (;;) {
    before = publish_sequence;
    after = before;
    if ((before & 1U) != 0U)
      continue;
    seen = frame_seen;
    timestamp = last_frame_ms;
    for (i = 0U; i < AUV_RC_FRAME_SIZE; ++i)
      frame[i] = published[i];
    after = publish_sequence;
    if (before == after && (after & 1U) == 0U) break;
  }

  return ((seen != 0U) && ((uint32_t)(now_ms - timestamp) <= AUV_RC_TIMEOUT_MS))
             ? 1U
             : 0U;
}

uint8_t AuvRcInput_AcceptCrc(uint32_t sequence, const uint8_t *frame,
                            uint8_t deadman, uint32_t now_ms)
{
    uint32_t i;
    if (frame == NULL || frame[0] != AUV_RC_FRAME_HEADER || deadman > 1U ||
        frame[6] > 2U || frame[7] > 2U ||
        frame[8] > 1U || frame[9] > 1U || frame[10] > 1U ||
        (frame[10] && frame[7] != 1U) ||
        (!frame[10] && frame[5] != 127U) ||
        (crc_seen && (int32_t)(sequence - crc_sequence) <= 0)) return 0U;
    ++publish_sequence;
    for (i=0U; i<AUV_RC_FRAME_SIZE; ++i) published[i]=frame[i];
    crc_sequence=sequence; crc_seen=1U; crc_deadman=deadman;
    last_frame_ms=now_ms; frame_seen=1U;
    ++publish_sequence;
    return 1U;
}

uint8_t AuvRcInput_CanArm(uint32_t now_ms)
{
    uint8_t frame[AUV_RC_FRAME_SIZE];
    uint32_t i;
    if (!crc_deadman || !AuvRcInput_CopyFreshFrame(now_ms,frame)) return 0U;
    for (i=1U;i<=4U;++i)
        if (frame[i] < 115U || frame[i] > 139U) return 0U;
    return 1U;
}
