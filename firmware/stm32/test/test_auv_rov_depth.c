#include "AuvRovDepth.h"
#include "AuvRovConfig.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static int line(AuvM10Parser *p, const char *text, float *depth)
{
    int result = 0;
    while (*text) result = AuvM10_Push(p, (uint8_t)*text++, depth);
    return result;
}
int main(void)
{
    AuvM10Parser parser;
    AuvRovDepthControl control;
    AuvDepthSample sample = {1.0f, 1U, 100U, 1U};
    float depth = -1.0f;
    float output;
    unsigned i;
    AuvM10_Reset(&parser);
    assert(line(&parser, "Depth:1.21m Temp:25.27C\r\n", &depth) == 1);
    assert(fabsf(depth - 1.21f) < 0.001f);
    assert(line(&parser, "Depth:nanm Temp:20C\n", &depth) == -1);
    assert(line(&parser, "Depth:1e1m Temp:20C\n", &depth) == -1);
    assert(line(&parser, "Depth:m Temp:20C\n", &depth) == -1);
    assert(line(&parser, "Depth:11111111111m Temp:20C\n", &depth) == -1);
    assert(line(&parser, "Depth: 1.5m Temp:-2.25C\n", &depth) == 1);
    assert(line(&parser, "Depth:badm Temp:20C\n", &depth) == -1);
    assert(line(&parser, "Depth:21m Temp:20C\n", &depth) == -1);
    assert(line(&parser, "Depth:2m Temp:20Cjunk\n", &depth) == -1);
    for (i = 0; i < 120; ++i) (void)AuvM10_Push(&parser, 'x', &depth);
    assert(AuvM10_Push(&parser, '\n', &depth) == -1);
    assert(line(&parser, "Depth:2m Temp=20C\n", &depth) == 1);
    assert(line(&parser, "Depth:-0.22m Temp:22.71C\n", &depth) == 1);
    assert(fabsf(depth+0.22f)<0.001f);
    AuvRovDepth_Reset(&control);
    assert(AuvRovDepth_Step(&control, &sample, 1, 1, 0) == 0);
    sample.depth_m = 1.2f;
    sample.sample_sequence++;
    sample.last_update_ms += 100U;
    output = AuvRovDepth_Step(&control, &sample, 1, 1, 0);
    assert(output > 0); /* too deep -> up */
    for (i = 0; i < 100; ++i)
        assert(AuvRovDepth_Step(&control, &sample, 1, 1, 0) == output);
    assert(AuvRovDepth_Step(&control, &sample, 0, 1, 0) == 0);
    assert(control.active == 0 && control.integral == 0);
    sample.depth_m=-0.22f;
    assert(AuvRovDepth_Step(&control,&sample,1,1,0)==0);
    sample.depth_m=-0.12f;sample.sample_sequence++;sample.last_update_ms+=2000;
    assert(fabsf(AuvRovDepth_Step(&control,&sample,1,1,0)-9.6f)<0.001f);
    assert(fabsf(control.integral+0.2f)<0.001f);
    /* Persistent sinking learns upward buoyancy compensation, bounded. */
    for(i=0;i<100;i++) {
        sample.sample_sequence++; sample.last_update_ms+=1000;
        output=AuvRovDepth_Step(&control,&sample,1,1,0);
        assert(output>0 && output<=AUV_ROV_DEPTH_OUTPUT_LIMIT);
        assert(fabsf(control.integral*AUV_ROV_DEPTH_KI)<=AUV_ROV_DEPTH_INTEGRAL_OUTPUT_LIMIT+0.001f);
    }
    sample.depth_m=control.target; sample.sample_sequence++; sample.last_update_ms+=1000;
    output=AuvRovDepth_Step(&control,&sample,1,1,0);
    assert(fabsf(output-AUV_ROV_DEPTH_INTEGRAL_OUTPUT_LIMIT)<0.001f);
    /* Long sensor gaps do not accrue integral; saturation does not wind up. */
    sample.depth_m=control.target+2; sample.sample_sequence++; sample.last_update_ms+=5000;
    {float integral=control.integral;
     assert(AuvRovDepth_Step(&control,&sample,1,1,0)==AUV_ROV_DEPTH_OUTPUT_LIMIT);
     assert(control.integral==integral);
     sample.sample_sequence++; sample.last_update_ms+=1000;
     assert(AuvRovDepth_Step(&control,&sample,1,1,0)==AUV_ROV_DEPTH_OUTPUT_LIMIT);
     assert(control.integral==integral);}
    sample.depth_m=control.target-0.2f; sample.sample_sequence++; sample.last_update_ms+=1000;
    AuvRovDepth_Step(&control,&sample,1,1,0);
    assert(fabsf(control.integral*AUV_ROV_DEPTH_KI)<AUV_ROV_DEPTH_INTEGRAL_OUTPUT_LIMIT);
    sample.valid=0;
    assert(AuvRovDepth_Step(&control,&sample,1,1,0)==0);
    assert(!control.active && control.integral==0);
    sample.valid=1;
    assert(AuvRovDepth_Step(&control,&sample,0,1,0)==0);
    /* Manual heave releases the old target; recenter captures the new one. */
    sample.depth_m=0.8f;sample.sample_sequence++;sample.last_update_ms+=2000;
    assert(AuvRovDepth_Step(&control,&sample,1,1,0)==0);
    assert(fabsf(control.target-0.8f)<0.001f);
    sample.depth_m=0.7f;sample.sample_sequence++;sample.last_update_ms+=2000;
    assert(AuvRovDepth_Step(&control,&sample,1,1,0)<0);
    assert(AuvRovDepth_Step(&control,&sample,0,1,0)==0);
    sample.depth_m=1.2f;
    assert(AuvRovDepth_Step(&control, &sample, 1, 0, NAN) == 0);
    assert(AuvRovDepth_Step(&control, &sample, 1, 0, 20) == -400);
    sample.valid = 0;
    assert(AuvRovDepth_Step(&control, &sample, 1, 0, 2) == 0);
    assert(!control.active && control.integral==0);
    return 0;
}
