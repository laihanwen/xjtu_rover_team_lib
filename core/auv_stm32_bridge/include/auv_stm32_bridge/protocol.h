// Copyright 2026 hanwen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUV_STM32_BRIDGE__PROTOCOL_H_
#define AUV_STM32_BRIDGE__PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define AUV_PROTOCOL_SYNC_0 UINT8_C(0xAA)
#define AUV_PROTOCOL_SYNC_1 UINT8_C(0x55)
#define AUV_PROTOCOL_VERSION UINT8_C(0x01)
#define AUV_PROTOCOL_MAX_PAYLOAD_SIZE 64U
#define AUV_PROTOCOL_HEADER_SIZE 5U
#define AUV_PROTOCOL_CRC_SIZE 2U
#define AUV_PROTOCOL_FRAME_OVERHEAD (AUV_PROTOCOL_HEADER_SIZE + AUV_PROTOCOL_CRC_SIZE)
#define AUV_PROTOCOL_MAX_FRAME_SIZE \
  (AUV_PROTOCOL_FRAME_OVERHEAD + AUV_PROTOCOL_MAX_PAYLOAD_SIZE)

typedef enum auv_protocol_message_type
{
  AUV_PROTOCOL_MSG_HEARTBEAT = 0x01,
  AUV_PROTOCOL_MSG_SET_ARMED = 0x02,
  AUV_PROTOCOL_MSG_MOTION_TARGET = 0x03,
  AUV_PROTOCOL_MSG_ACTUATOR_COMMAND = 0x04,
  AUV_PROTOCOL_MSG_ACK = 0x7F,
  AUV_PROTOCOL_MSG_STATUS = 0x80,
  AUV_PROTOCOL_MSG_IMU = 0x81,
  AUV_PROTOCOL_MSG_DEPTH = 0x82,
  AUV_PROTOCOL_MSG_ACTUATOR_STATUS = 0x83
} auv_protocol_message_type_t;

uint16_t auv_protocol_crc16_ccitt_false(const uint8_t * data, size_t size);

int auv_protocol_is_known_message_type(uint8_t message_type);

size_t auv_protocol_encode_frame(
  uint8_t message_type, const uint8_t * payload, size_t payload_size,
  uint8_t * output, size_t output_capacity);

void auv_protocol_write_u16_le(uint8_t * output, uint16_t value);
void auv_protocol_write_i16_le(uint8_t * output, int16_t value);
void auv_protocol_write_u32_le(uint8_t * output, uint32_t value);
void auv_protocol_write_f32_le(uint8_t * output, float value);
uint16_t auv_protocol_read_u16_le(const uint8_t * input);
int16_t auv_protocol_read_i16_le(const uint8_t * input);
uint32_t auv_protocol_read_u32_le(const uint8_t * input);
float auv_protocol_read_f32_le(const uint8_t * input);

#ifdef __cplusplus
}
#endif

#endif  // AUV_STM32_BRIDGE__PROTOCOL_H_
