#include "AuvRovDepth.h"
#include "AuvRovConfig.h"
#include <string.h>
#include <math.h>

/* M10 emits bounded decimal ASCII, not general C floating-point literals. */
static uint8_t ParseDecimal(char *cursor, char **end, float *value)
{
    float number=0.0f, fraction=0.1f;
    uint8_t negative=0U, point=0U, digits=0U;
    while (*cursor==' ') ++cursor;
    if (*cursor=='-' || *cursor=='+') {negative=(*cursor=='-');++cursor;}
    while ((*cursor>='0' && *cursor<='9') || *cursor=='.') {
        if (*cursor=='.') {if (point) return 0U;point=1U;}
        else {
            if (++digits > 10U) return 0U;
            if (point) {number+=(float)(*cursor-'0')*fraction;fraction*=0.1f;}
            else number=number*10.0f+(float)(*cursor-'0');
        }
        ++cursor;
    }
    if (!digits) return 0U;
    *end=cursor;*value=negative ? -number : number;
    return 1U;
}

void AuvM10_Reset(AuvM10Parser *parser)
{
    memset(parser, 0, sizeof(*parser));
}

int AuvM10_Push(AuvM10Parser *parser, uint8_t byte, float *depth)
{
    char *end;
    char *cursor;
    float value;
    float temperature;
    if (byte != '\n') {
        if (parser->length >= sizeof(parser->line) - 1U || byte == 0U)
            parser->overflow = 1U;
        else if (parser->overflow == 0U)
            parser->line[parser->length++] = (char)byte;
        return 0;
    }
    parser->line[parser->length] = '\0';
    parser->length = 0U;
    if (parser->overflow != 0U) {
        parser->overflow = 0U;
        return -1;
    }
    if (strncmp(parser->line, "Depth:", 6U) != 0) return -1;
    cursor = parser->line + 6;
    if (!ParseDecimal(cursor, &end, &value)) return -1;
    if (*end != 'm' || !isfinite(value) ||
        value < 0.0f || value > AUV_DEPTH_MAX_METERS) return -1;
    cursor = end + 1;
    while (*cursor == ' ') ++cursor;
    if (strncmp(cursor, "Temp:", 5U) != 0 &&
        strncmp(cursor, "Temp=", 5U) != 0) return -1;
    cursor += 5;
    if (!ParseDecimal(cursor, &end, &temperature)) return -1;
    if (*end != 'C' || !isfinite(temperature)) return -1;
    cursor = end + 1;
    while (*cursor == ' ' || *cursor == '\r') ++cursor;
    if (*cursor != '\0') return -1;
    *depth = value;
    return 1;
}

void AuvRovDepth_Reset(AuvRovDepthControl *control)
{
    memset(control, 0, sizeof(*control));
}

float AuvRovDepth_Step(AuvRovDepthControl *control, const AuvDepthSample *sample,
                      uint8_t enabled, uint8_t lock_current, float target)
{
    float error;
    float dt;
    float proposed;
    float output;
    if (!enabled || !sample->valid || !isfinite(sample->depth_m) ||
        sample->depth_m < 0.0f || sample->depth_m > AUV_DEPTH_MAX_METERS ||
        (!lock_current && (!isfinite(target) || target < 0.0f ||
                           target > AUV_DEPTH_MAX_METERS))) {
        AuvRovDepth_Reset(control);
        return 0.0f;
    }
    if (!control->active) {
        control->target = lock_current ? sample->depth_m : target;
        control->sequence = sample->sample_sequence - 1U;
        control->sample_ms = sample->last_update_ms;
        control->active = 1U;
    }
    if (!lock_current) control->target = target;
    /* Integral advances only for a new sensor sample, with real sample dt. */
    if (sample->sample_sequence == control->sequence) return control->output;
    dt = (float)(uint32_t)(sample->last_update_ms - control->sample_ms) * 0.001f;
    if (dt > (float)AUV_DEPTH_TIMEOUT_MS * 0.001f) dt = 0.0f;
    error = control->target - sample->depth_m;
    proposed = control->integral + error * dt;
    output = AUV_DEPTH_KP * error + AUV_DEPTH_KI * proposed;
    /* Conditional integration prevents windup into output saturation. */
    if (fabsf(output) <= AUV_DEPTH_OUTPUT_LIMIT || output * error < 0.0f)
        control->integral = proposed;
    output = AUV_DEPTH_KP * error + AUV_DEPTH_KI * control->integral;
    if (output > AUV_DEPTH_OUTPUT_LIMIT) output = AUV_DEPTH_OUTPUT_LIMIT;
    if (output < -AUV_DEPTH_OUTPUT_LIMIT) output = -AUV_DEPTH_OUTPUT_LIMIT;
    control->output = AUV_DEPTH_FZ_SIGN * output;
    control->sequence = sample->sample_sequence;
    control->sample_ms = sample->last_update_ms;
    return control->output;
}
