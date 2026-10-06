#ifndef AUV_WATER_PD_H
#define AUV_WATER_PD_H
#include <math.h>
#include <stdint.h>
typedef struct { uint32_t ms, sequence; float angle, rate; uint8_t ready; } AuvWaterRate;
static inline float AuvWater_Wrap(float x)
{
    if(!isfinite(x))return 0;
    while(x>180)x-=360;
    while(x< -180)x+=360;
    return x;
}
static inline void AuvWater_Rate(AuvWaterRate *s,float angle,uint32_t ms,uint32_t sequence)
{
    uint32_t elapsed;
    if(s->ready && sequence==s->sequence)return;
    elapsed=(uint32_t)(ms-s->ms);
    if(!s->ready || elapsed<2 || elapsed>100) s->rate=0;
    else {
        float dt=elapsed*0.001f;
        float r=AuvWater_Wrap(angle-s->angle)/dt;
        if(r>200)r=200;if(r< -200)r=-200;
        s->rate+=dt/(0.08f+dt)*(r-s->rate);
    }
    s->angle=angle;s->ms=ms;s->sequence=sequence;s->ready=1;
}
static inline float AuvWater_Pd(float error,float rate,float p,float d,float limit)
{
    float out=p*error-d*rate;
    if(!isfinite(out))return 0;
    if(out>limit)out=limit;if(out< -limit)out=-limit;
    return out;
}
static inline uint8_t AuvWater_Heading(float angle,uint8_t allowed,uint8_t turning,
                                     uint8_t *locked,float *target)
{
    if(!allowed || turning) {*locked=0;*target=angle;return 0;}
    if(!*locked){*target=angle;*locked=1;}
    return 1;
}
#endif
