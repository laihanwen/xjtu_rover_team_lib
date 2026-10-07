#include "AuvWaterPd.h"
#include <assert.h>
int main(void)
{
    AuvWaterRate r={0};uint8_t lock=0;float target=0;
    assert(AuvWater_Heading(30,1,0,&lock,&target) && target==30);
    assert(AuvWater_Heading(40,1,0,&lock,&target) && target==30);
    assert(!AuvWater_Heading(50,1,1,&lock,&target) && !lock);
    assert(AuvWater_Heading(55,1,0,&lock,&target) && target==55);
    assert(!AuvWater_Heading(60,0,0,&lock,&target) && !lock);
    assert(AuvWater_Wrap(179-(-179))==-2);
    AuvWater_Rate(&r,179,100,1);assert(r.rate==0);
    AuvWater_Rate(&r,-179,110,2);assert(r.rate>0 && r.rate<25);
    float rate=r.rate;
    AuvWater_Rate(&r,-179,110,2);assert(r.rate==rate);
    AuvWater_Rate(&r,0,500,3);assert(r.rate==0);
    assert(AuvWater_Pd(0,10,0.75f,0.15f,20)<0);
    assert(AuvWater_Pd(100,0,1,0.1f,20)==20);
    assert(AuvWater_Pd(-100,0,1,0.1f,20)==-20);
    return 0;
}
