#include "AuvProtocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static const uint8_t heartbeat_golden[] = {
    0xAA, 0x55, 0x01, 0x01, 0x08, 0x04, 0x03, 0x02,
    0x01, 0x0D, 0x0C, 0x0B, 0x0A, 0x55, 0x8D
};

static void TestGoldenVector(void)
{
    const uint8_t payload[] = {0x04, 0x03, 0x02, 0x01, 0x0D, 0x0C, 0x0B, 0x0A};
    uint8_t encoded[AUV_PROTOCOL_MAX_FRAME_SIZE];
    size_t size = AuvProtocol_Encode(
        AUV_MSG_HEARTBEAT, payload, sizeof(payload), encoded, sizeof(encoded));
    assert(size == sizeof(heartbeat_golden));
    assert(memcmp(encoded, heartbeat_golden, size) == 0);
    assert(AuvProtocol_Crc16((const uint8_t *)"123456789", 9U) == 0x29B1U);
    {
        const uint8_t level[]={4,3,2,1,1};
        const uint8_t golden[]={0xaa,0x55,1,8,5,4,3,2,1,1,0xeb,0xf9};
        AuvProtocolParser parser;
        AuvProtocolFrame frame={0};
        size=AuvProtocol_Encode(AUV_MSG_CALIBRATE_LEVEL,level,sizeof(level),encoded,sizeof(encoded));
        assert(size==sizeof(golden) && memcmp(encoded,golden,size)==0);
        AuvProtocolParser_Init(&parser);
        for(size_t i=0;i<size;++i) {
            AuvParseResult result=AuvProtocolParser_Push(&parser,encoded[i],&frame);
            assert(result==(i+1==size ? AUV_PARSE_FRAME_READY : AUV_PARSE_INCOMPLETE));
        }
        assert(frame.message_type==AUV_MSG_CALIBRATE_LEVEL && frame.payload_length==5);
    }
}

static void TestGripperGoldenVector(void)
{
    const uint8_t payload[] = {
        0x04, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0x80, 0x3F
    };
    const uint8_t expected[] = {
        0xAA, 0x55, 0x01, 0x04, 0x09, 0x04, 0x03, 0x02,
        0x01, 0x01, 0x00, 0x00, 0x80, 0x3F, 0x69, 0xF4
    };
    uint8_t encoded[AUV_PROTOCOL_MAX_FRAME_SIZE];
    size_t size = AuvProtocol_Encode(
        AUV_MSG_ACTUATOR_COMMAND, payload, sizeof(payload), encoded,
        sizeof(encoded));
    assert(size == sizeof(expected));
    assert(memcmp(encoded, expected, size) == 0);
}

static void TestParserAndRecovery(void)
{
    AuvProtocolParser parser;
    AuvProtocolFrame frame;
    uint8_t corrupted[sizeof(heartbeat_golden)];
    size_t i;

    AuvProtocolParser_Init(&parser);
    for (i = 0U; i < sizeof(heartbeat_golden); ++i) {
        AuvParseResult result = AuvProtocolParser_Push(
            &parser, heartbeat_golden[i], &frame);
        assert(result == ((i + 1U == sizeof(heartbeat_golden))
            ? AUV_PARSE_FRAME_READY : AUV_PARSE_INCOMPLETE));
    }
    assert(frame.message_type == AUV_MSG_HEARTBEAT);
    assert(frame.payload_length == 8U);

    memcpy(corrupted, heartbeat_golden, sizeof(corrupted));
    corrupted[7] ^= 0x80U;
    for (i = 0U; i < sizeof(corrupted); ++i)
        (void)AuvProtocolParser_Push(&parser, corrupted[i], &frame);
    assert(parser.length == 0U);
    for (i = 0U; i < sizeof(heartbeat_golden); ++i) {
        AuvParseResult result = AuvProtocolParser_Push(
            &parser, heartbeat_golden[i], &frame);
        if (i + 1U == sizeof(heartbeat_golden))
            assert(result == AUV_PARSE_FRAME_READY);
    }
}

static void TestRejectsInvalidHeaders(void)
{
    AuvProtocolParser parser;
    AuvProtocolFrame frame;
    const uint8_t invalid[] = {0xAA, 0x55, 0x02, 0x01, 0x00};
    size_t i;

    AuvProtocolParser_Init(&parser);
    for (i = 0U; i < sizeof(invalid); ++i) {
        AuvParseResult result = AuvProtocolParser_Push(&parser, invalid[i], &frame);
        if (i + 1U == sizeof(invalid)) assert(result == AUV_PARSE_REJECTED);
    }
    assert(parser.length == 0U);
}

int main(void)
{
    TestGoldenVector();
    TestGripperGoldenVector();
    TestParserAndRecovery();
    TestRejectsInvalidHeaders();
    puts("AuvProtocol host tests passed");
    return 0;
}
