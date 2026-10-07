#include "AuvThrusterDeadband.h"
#include <assert.h>
int main(void)
{
    assert(AuvThruster_Compensate(0,75,48)==0);
    assert(AuvThruster_Compensate(0.9f,75,48)==0);
    assert(AuvThruster_Compensate(NAN,75,48)==0);
    assert(AuvThruster_Compensate(75,75,48)==75);
    assert(AuvThruster_Compensate(-100,75,48)==-75);
    for(int i=1;i<=75;i++) {
        float p=AuvThruster_Compensate((float)i,75,48);
        assert(p>=48 && p<=75);
        assert(p==-AuvThruster_Compensate((float)-i,75,48));
        if(i>1) assert(p>AuvThruster_Compensate((float)(i-1),75,48));
    }
    assert(!AuvAttitude_Active(1.9f,0));
    assert(AuvAttitude_Active(-2,0));
    assert(AuvAttitude_Active(1.5f,1));
    assert(!AuvAttitude_Active(0.9f,1));
    assert(AuvThruster_Slew(0,48,2)==2);
    assert(AuvThruster_Slew(48,-48,2)==46);
    assert(AuvThruster_Slew(1,0,2)==0);
    assert(AuvThruster_Slew(0,48,0)==0);
    return 0;
}
