/**
 * @file AuvProtocol.c
 * @brief Portable AUV serial protocol v1 implementation.
 */

#include "AuvProtocol.h"

#include <string.h>

uint16_t AuvProtocol_Crc16(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFFU;
    size_t index;
    uint8_t bit;

    if ((data == NULL) && (size != 0U)) return 0U;
    for (index = 0U; index < size; ++index) {
        crc ^= (uint16_t)((uint16_t)data[index] << 8U);
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 0x8000U) != 0U)
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

uint8_t AuvProtocol_IsKnownType(uint8_t message_type)
{
    switch (message_type) {
    case AUV_MSG_HEARTBEAT:
    case AUV_MSG_SET_ARMED:
    case AUV_MSG_MOTION_TARGET:
    case AUV_MSG_ACTUATOR_COMMAND:
    case AUV_MSG_RC_TARGET:
    case AUV_MSG_REMOTE_KILL:
    case 0x07U: /* Single-thruster commissioning pulse. */
    case AUV_MSG_CALIBRATE_LEVEL: /* Explicit DISARM shore level calibration. */
    case AUV_MSG_ACK:
    case AUV_MSG_STATUS:
    case AUV_MSG_IMU:
    case AUV_MSG_DEPTH:
    case AUV_MSG_ACTUATOR_STATUS:
    case AUV_MSG_PID_DIAGNOSTIC:
        return 1U;
    default:
        return 0U;
    }
}

size_t AuvProtocol_Encode(uint8_t message_type,
                          const uint8_t *payload,
                          size_t payload_size,
                          uint8_t *output,
                          size_t output_capacity)
{
    size_t frame_size = AUV_PROTOCOL_FRAME_OVERHEAD + payload_size;
    uint16_t crc;

    if ((output == NULL) ||
        ((payload == NULL) && (payload_size != 0U)) ||
        (payload_size > AUV_PROTOCOL_MAX_PAYLOAD_SIZE) ||
        (output_capacity < frame_size) ||
        (AuvProtocol_IsKnownType(message_type) == 0U)) return 0U;

    output[0] = AUV_PROTOCOL_SYNC_0;
    output[1] = AUV_PROTOCOL_SYNC_1;
    output[2] = AUV_PROTOCOL_VERSION;
    output[3] = message_type;
    output[4] = (uint8_t)payload_size;
    if (payload_size != 0U) memcpy(&output[5], payload, payload_size);
    crc = AuvProtocol_Crc16(&output[2], 3U + payload_size);
    AuvProtocol_WriteU16Le(&output[5U + payload_size], crc);
    return frame_size;
}

void AuvProtocolParser_Init(AuvProtocolParser *parser)
{
    if (parser != NULL) parser->length = 0U;
}

static void ParserResync(AuvProtocolParser *parser)
{
    uint8_t index;
    uint8_t next = parser->length;

    for (index = 1U; index + 1U < parser->length; ++index) {
        if ((parser->buffer[index] == AUV_PROTOCOL_SYNC_0) &&
            (parser->buffer[index + 1U] == AUV_PROTOCOL_SYNC_1)) {
            next = index;
            break;
        }
    }
    if (next < parser->length) {
        uint8_t remaining = (uint8_t)(parser->length - next);
        memmove(parser->buffer, &parser->buffer[next], remaining);
        parser->length = remaining;
    } else if (parser->buffer[parser->length - 1U] == AUV_PROTOCOL_SYNC_0) {
        parser->buffer[0] = AUV_PROTOCOL_SYNC_0;
        parser->length = 1U;
    } else {
        parser->length = 0U;
    }
}

AuvParseResult AuvProtocolParser_Push(AuvProtocolParser *parser,
                                      uint8_t byte,
                                      AuvProtocolFrame *frame)
{
    uint8_t payload_size;
    uint8_t frame_size;
    uint16_t expected_crc;
    uint16_t actual_crc;

    if ((parser == NULL) || (frame == NULL)) return AUV_PARSE_REJECTED;
    if ((parser->length == 0U) && (byte != AUV_PROTOCOL_SYNC_0))
        return AUV_PARSE_INCOMPLETE;
    if ((parser->length == 1U) && (byte != AUV_PROTOCOL_SYNC_1)) {
        parser->length = (byte == AUV_PROTOCOL_SYNC_0) ? 1U : 0U;
        return AUV_PARSE_INCOMPLETE;
    }
    if (parser->length >= AUV_PROTOCOL_MAX_FRAME_SIZE) {
        ParserResync(parser);
        return AUV_PARSE_REJECTED;
    }
    parser->buffer[parser->length++] = byte;
    if (parser->length < 5U) return AUV_PARSE_INCOMPLETE;

    payload_size = parser->buffer[4];
    if ((parser->buffer[2] != AUV_PROTOCOL_VERSION) ||
        (AuvProtocol_IsKnownType(parser->buffer[3]) == 0U) ||
        (payload_size > AUV_PROTOCOL_MAX_PAYLOAD_SIZE)) {
        ParserResync(parser);
        return AUV_PARSE_REJECTED;
    }
    frame_size = (uint8_t)(AUV_PROTOCOL_FRAME_OVERHEAD + payload_size);
    if (parser->length < frame_size) return AUV_PARSE_INCOMPLETE;

    expected_crc = AuvProtocol_ReadU16Le(&parser->buffer[5U + payload_size]);
    actual_crc = AuvProtocol_Crc16(&parser->buffer[2], 3U + payload_size);
    if (expected_crc != actual_crc) {
        ParserResync(parser);
        return AUV_PARSE_REJECTED;
    }
    frame->message_type = parser->buffer[3];
    frame->payload_length = payload_size;
    if (payload_size != 0U) memcpy(frame->payload, &parser->buffer[5], payload_size);
    parser->length = 0U;
    return AUV_PARSE_FRAME_READY;
}

void AuvProtocol_WriteU16Le(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)(value >> 8U);
}

void AuvProtocol_WriteI16Le(uint8_t *output, int16_t value)
{
    AuvProtocol_WriteU16Le(output, (uint16_t)value);
}

void AuvProtocol_WriteU32Le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8U) & 0xFFU);
    output[2] = (uint8_t)((value >> 16U) & 0xFFU);
    output[3] = (uint8_t)(value >> 24U);
}

void AuvProtocol_WriteF32Le(uint8_t *output, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    AuvProtocol_WriteU32Le(output, bits);
}

uint16_t AuvProtocol_ReadU16Le(const uint8_t *input)
{
    return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

uint32_t AuvProtocol_ReadU32Le(const uint8_t *input)
{
    return (uint32_t)input[0] |
        ((uint32_t)input[1] << 8U) |
        ((uint32_t)input[2] << 16U) |
        ((uint32_t)input[3] << 24U);
}

float AuvProtocol_ReadF32Le(const uint8_t *input)
{
    uint32_t bits = AuvProtocol_ReadU32Le(input);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}
