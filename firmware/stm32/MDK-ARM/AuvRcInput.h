/**
 * @file AuvRcInput.h
 * @brief Interrupt-safe receiver for the legacy 0xA5 remote-control frame.
 */

#ifndef AUV_RC_INPUT_H
#define AUV_RC_INPUT_H

#include <stdint.h>

#define AUV_RC_FRAME_HEADER 0xA5U
#define AUV_RC_FRAME_SIZE 11U
#define AUV_RC_TIMEOUT_MS 250U

void AuvRcInput_Init(void);
uint8_t AuvRcInput_AcceptCrc(uint32_t sequence, const uint8_t *frame,
                            uint8_t deadman, uint32_t now_ms);
uint8_t AuvRcInput_CanArm(uint32_t now_ms);
void AuvRcInput_PushByte(uint8_t byte, uint32_t now_ms);
uint8_t AuvRcInput_CopyFreshFrame(uint32_t now_ms,
                                  uint8_t frame[AUV_RC_FRAME_SIZE]);

#endif
