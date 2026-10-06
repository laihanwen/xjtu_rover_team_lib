/**
 * @file AuvMotionTarget.c
 * @brief Validated, timeout-protected Pi motion target storage.
 */

#include "AuvMotionTarget.h"

#include <float.h>
#include <stddef.h>

#include "AuvProtocol.h"

#define AUV_MOTION_MAX_ABS_VELOCITY_MPS 5.0f
#define AUV_MOTION_MAX_DEPTH_M 100.0f
#define AUV_MOTION_PI 3.141592654f

static volatile AuvMotionTarget published;
static volatile uint32_t last_target_ms;
static volatile uint32_t publish_generation;
static volatile uint8_t target_seen;

static uint8_t FiniteInRange(float value, float minimum, float maximum) {
  return ((value == value) && (value >= minimum) && (value <= maximum) &&
          (value >= -FLT_MAX) && (value <= FLT_MAX))
             ? 1U
             : 0U;
}

static uint8_t SequenceNewer(uint32_t incoming, uint32_t previous) {
  return ((int32_t)(incoming - previous) > 0) ? 1U : 0U;
}

void AuvMotionTarget_Init(void) {
  published.sequence = 0U;
  published.vx = 0.0f;
  published.vy = 0.0f;
  published.depth = 0.0f;
  published.yaw = 0.0f;
  last_target_ms = 0U;
  publish_generation = 0U;
  target_seen = 0U;
}

AuvArmResult AuvMotionTarget_Accept(const uint8_t *payload,
                                    uint8_t payload_length, uint32_t now_ms,
                                    uint8_t armed) {
  AuvMotionTarget incoming;

  if ((payload == NULL) || (payload_length != 20U))
    return AUV_ARM_MALFORMED;
  incoming.sequence = AuvProtocol_ReadU32Le(&payload[0]);
  incoming.vx = AuvProtocol_ReadF32Le(&payload[4]);
  incoming.vy = AuvProtocol_ReadF32Le(&payload[8]);
  incoming.depth = AuvProtocol_ReadF32Le(&payload[12]);
  incoming.yaw = AuvProtocol_ReadF32Le(&payload[16]);
  if ((FiniteInRange(incoming.vx, -AUV_MOTION_MAX_ABS_VELOCITY_MPS,
                     AUV_MOTION_MAX_ABS_VELOCITY_MPS) == 0U) ||
      (FiniteInRange(incoming.vy, -AUV_MOTION_MAX_ABS_VELOCITY_MPS,
                     AUV_MOTION_MAX_ABS_VELOCITY_MPS) == 0U) ||
      (FiniteInRange(incoming.depth, 0.0f, AUV_MOTION_MAX_DEPTH_M) == 0U) ||
      (FiniteInRange(incoming.yaw, -AUV_MOTION_PI, AUV_MOTION_PI) == 0U))
    return AUV_ARM_MALFORMED;
  if ((target_seen != 0U) &&
      (SequenceNewer(incoming.sequence, published.sequence) == 0U))
    return AUV_ARM_MALFORMED;

  ++publish_generation;
  published = incoming;
  last_target_ms = now_ms;
  target_seen = 1U;
  ++publish_generation;
  return (armed != 0U) ? AUV_ARM_ACCEPTED : AUV_ARM_DISARMED;
}

uint8_t AuvMotionTarget_CopyFresh(uint32_t now_ms, AuvMotionTarget *target) {
  uint32_t before;
  uint32_t after;
  uint32_t timestamp;
  uint8_t seen;

  if (target == NULL)
    return 0U;
  for (;;) {
    before = publish_generation;
    after = before;
    if ((before & 1U) != 0U)
      continue;
    seen = target_seen;
    timestamp = last_target_ms;
    *target = published;
    after = publish_generation;
    if (before == after && (after & 1U) == 0U) break;
  }
  return ((seen != 0U) &&
          ((uint32_t)(now_ms - timestamp) <= AUV_MOTION_TARGET_TIMEOUT_MS))
             ? 1U
             : 0U;
}
