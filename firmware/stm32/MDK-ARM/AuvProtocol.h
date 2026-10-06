/**
 * @file AuvProtocol.h
 * @brief Portable, allocation-free AUV serial protocol v1.
 */

#ifndef AUV_PROTOCOL_H
#define AUV_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define AUV_PROTOCOL_SYNC_0            0xAAU
#define AUV_PROTOCOL_SYNC_1            0x55U
#define AUV_PROTOCOL_VERSION           0x01U
#define AUV_PROTOCOL_MAX_PAYLOAD_SIZE  64U
#define AUV_PROTOCOL_FRAME_OVERHEAD    7U
#define AUV_PROTOCOL_MAX_FRAME_SIZE    71U

typedef enum {
    AUV_MSG_HEARTBEAT       = 0x01,
    AUV_MSG_SET_ARMED       = 0x02,
    AUV_MSG_MOTION_TARGET   = 0x03,
    AUV_MSG_ACTUATOR_COMMAND = 0x04,
    AUV_MSG_RC_TARGET       = 0x05,
    AUV_MSG_REMOTE_KILL     = 0x06,
    AUV_MSG_CALIBRATE_LEVEL = 0x08,
    AUV_MSG_ACK             = 0x7F,
    AUV_MSG_STATUS          = 0x80,
    AUV_MSG_IMU             = 0x81,
    AUV_MSG_DEPTH           = 0x82,
    AUV_MSG_ACTUATOR_STATUS = 0x83,
    AUV_MSG_PID_DIAGNOSTIC = 0x84
} AuvMessageType;

typedef struct {
    uint8_t message_type;
    uint8_t payload_length;
    uint8_t payload[AUV_PROTOCOL_MAX_PAYLOAD_SIZE];
} AuvProtocolFrame;

typedef struct {
    uint8_t buffer[AUV_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t length;
} AuvProtocolParser;

typedef enum {
    AUV_PARSE_INCOMPLETE = 0,
    AUV_PARSE_FRAME_READY,
    AUV_PARSE_REJECTED
} AuvParseResult;

uint16_t AuvProtocol_Crc16(const uint8_t *data, size_t size);
uint8_t AuvProtocol_IsKnownType(uint8_t message_type);
size_t AuvProtocol_Encode(uint8_t message_type,
                          const uint8_t *payload,
                          size_t payload_size,
                          uint8_t *output,
                          size_t output_capacity);
void AuvProtocolParser_Init(AuvProtocolParser *parser);
AuvParseResult AuvProtocolParser_Push(AuvProtocolParser *parser,
                                      uint8_t byte,
                                      AuvProtocolFrame *frame);
void AuvProtocol_WriteU16Le(uint8_t *output, uint16_t value);
void AuvProtocol_WriteI16Le(uint8_t *output, int16_t value);
void AuvProtocol_WriteU32Le(uint8_t *output, uint32_t value);
void AuvProtocol_WriteF32Le(uint8_t *output, float value);
uint16_t AuvProtocol_ReadU16Le(const uint8_t *input);
uint32_t AuvProtocol_ReadU32Le(const uint8_t *input);
float AuvProtocol_ReadF32Le(const uint8_t *input);

#endif
