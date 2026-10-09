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

#include "auv_stm32_bridge/protocol.h"

#include <string.h>

uint16_t auv_protocol_crc16_ccitt_false(const uint8_t * data, const size_t size)
{
  uint16_t crc = UINT16_C(0xFFFF);
  size_t index;
  uint8_t bit;

  if (data == NULL && size != 0U) {
    return 0U;
  }
  for (index = 0U; index < size; ++index) {
    crc ^= (uint16_t)((uint16_t)data[index] << 8U);
    for (bit = 0U; bit < 8U; ++bit) {
      crc = (crc & UINT16_C(0x8000)) != 0U ?
        (uint16_t)((crc << 1U) ^ UINT16_C(0x1021)) : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

int auv_protocol_is_known_message_type(const uint8_t message_type)
{
  switch (message_type) {
    case AUV_PROTOCOL_MSG_HEARTBEAT:
    case AUV_PROTOCOL_MSG_SET_ARMED:
    case AUV_PROTOCOL_MSG_MOTION_TARGET:
    case AUV_PROTOCOL_MSG_ACTUATOR_COMMAND:
    case AUV_PROTOCOL_MSG_SELECT_MODE:
    case AUV_PROTOCOL_MSG_ACK:
    case AUV_PROTOCOL_MSG_STATUS:
    case AUV_PROTOCOL_MSG_IMU:
    case AUV_PROTOCOL_MSG_DEPTH:
    case AUV_PROTOCOL_MSG_ACTUATOR_STATUS:
      return 1;
    default:
      return 0;
  }
}

size_t auv_protocol_encode_frame(
  const uint8_t message_type, const uint8_t * payload, const size_t payload_size,
  uint8_t * output, const size_t output_capacity)
{
  size_t frame_size;
  uint16_t crc;

  if (output == NULL || payload_size > AUV_PROTOCOL_MAX_PAYLOAD_SIZE ||
    (payload == NULL && payload_size != 0U) ||
    auv_protocol_is_known_message_type(message_type) == 0)
  {
    return 0U;
  }
  frame_size = AUV_PROTOCOL_FRAME_OVERHEAD + payload_size;
  if (output_capacity < frame_size) {
    return 0U;
  }

  output[0] = AUV_PROTOCOL_SYNC_0;
  output[1] = AUV_PROTOCOL_SYNC_1;
  output[2] = AUV_PROTOCOL_VERSION;
  output[3] = message_type;
  output[4] = (uint8_t)payload_size;
  if (payload_size != 0U) {
    memcpy(&output[AUV_PROTOCOL_HEADER_SIZE], payload, payload_size);
  }
  crc = auv_protocol_crc16_ccitt_false(&output[2], 3U + payload_size);
  auv_protocol_write_u16_le(&output[AUV_PROTOCOL_HEADER_SIZE + payload_size], crc);
  return frame_size;
}

void auv_protocol_write_u16_le(uint8_t * output, const uint16_t value)
{
  output[0] = (uint8_t)(value & UINT16_C(0x00FF));
  output[1] = (uint8_t)(value >> 8U);
}

void auv_protocol_write_i16_le(uint8_t * output, const int16_t value)
{
  auv_protocol_write_u16_le(output, (uint16_t)value);
}

void auv_protocol_write_u32_le(uint8_t * output, const uint32_t value)
{
  output[0] = (uint8_t)(value & UINT32_C(0x000000FF));
  output[1] = (uint8_t)((value >> 8U) & UINT32_C(0x000000FF));
  output[2] = (uint8_t)((value >> 16U) & UINT32_C(0x000000FF));
  output[3] = (uint8_t)(value >> 24U);
}

void auv_protocol_write_f32_le(uint8_t * output, const float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  auv_protocol_write_u32_le(output, bits);
}

uint16_t auv_protocol_read_u16_le(const uint8_t * input)
{
  return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

int16_t auv_protocol_read_i16_le(const uint8_t * input)
{
  return (int16_t)auv_protocol_read_u16_le(input);
}

uint32_t auv_protocol_read_u32_le(const uint8_t * input)
{
  return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
         ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

float auv_protocol_read_f32_le(const uint8_t * input)
{
  const uint32_t bits = auv_protocol_read_u32_le(input);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}
