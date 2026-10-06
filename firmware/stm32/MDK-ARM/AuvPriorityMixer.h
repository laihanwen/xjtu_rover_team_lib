#ifndef AUV_PRIORITY_MIXER_H
#define AUV_PRIORITY_MIXER_H
#include <math.h>
/* Reserve attitude authority, uniformly reducing motion only. */
static inline float AuvPriority_Combine(const float motion[8], const float attitude[8],
                                       float limit, float output[8])
{
    float attitude_scale=1.0f, motion_scale=1.0f, peak=0.0f;
    int i;
    for(i=0;i<8;i++) {
        if(!isfinite(motion[i]) || !isfinite(attitude[i]) || !isfinite(limit) || limit<=0) {
            int j;for(j=0;j<8;j++)output[j]=0;return 0;
        }
        if(fabsf(attitude[i])>peak)peak=fabsf(attitude[i]);
    }
    if(peak>limit)attitude_scale=limit/peak;
    for(i=0;i<8;i++) {
        float a=attitude[i]*attitude_scale, available;
        if(motion[i]>0) available=(limit-a)/motion[i];
        else if(motion[i]<0) available=(-limit-a)/motion[i];
        else continue;
        if(available<motion_scale)motion_scale=available;
    }
    if(motion_scale<0)motion_scale=0;
    for(i=0;i<8;i++)output[i]=attitude[i]*attitude_scale+motion[i]*motion_scale;
    return motion_scale;
}
#endif
