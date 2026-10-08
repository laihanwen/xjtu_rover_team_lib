#include "AuvSampleAge.h"
#include <assert.h>
int main(void)
{
    assert(!AuvSample_IsFresh(0,100,100,250));
    assert(AuvSample_IsFresh(1,101,101,250));
    assert(!AuvSample_IsFresh(1,101,100,250)); /* old-time/new-sample race */
    assert(AuvSample_IsFresh(1,100,350,250));
    assert(!AuvSample_IsFresh(1,100,351,250));
    assert(AuvSample_IsFresh(1,0xfffffff0U,0x10U,250));
    return 0;
}
