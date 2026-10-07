#include "AuvPriorityMixer.h"
#include <assert.h>
int main(void)
{
    float m[8]={100,-100,80,-80,100,-100,80,-80};
    float a[8]={20,-20,-20,20,20,-20,-20,20}, out[8];
    float scale=AuvPriority_Combine(m,a,100,out);
    assert(fabsf(scale-0.8f)<0.00001f);
    for(int i=0;i<8;i++) {
        assert(fabsf(out[i])<=100.001f);
        assert(fabsf(out[i]-m[i]*scale-a[i])<0.0001f);
    }
    for(int i=0;i<8;i++)m[i]=0;
    AuvPriority_Combine(m,a,100,out);
    for(int i=0;i<8;i++)assert(out[i]==a[i]);
    for(int i=0;i<8;i++)a[i]*=10;
    AuvPriority_Combine(m,a,100,out);
    for(int i=0;i<8;i++)assert(fabsf(out[i])<=100);
    m[2]=NAN;
    AuvPriority_Combine(m,a,100,out);
    for(int i=0;i<8;i++)assert(out[i]==0);
    return 0;
}
