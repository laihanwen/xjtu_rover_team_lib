#include "AuvH30Frame.h"
#include <assert.h>
static void seal(uint8_t *f, uint16_t n)
{
    uint16_t i;
    uint8_t a = 0U, b = 0U;
    for (i = 2U; i < n - 2U; ++i) { a = (uint8_t)(a + f[i]); b = (uint8_t)(b + a); }
    f[n-2U] = a; f[n-1U] = b;
}
int main(void)
{
    AuvH30Frame p = {0};
    uint8_t f[9] = {0x59,0x53,0,0,2,0x40,0};
    uint16_t i;
    seal(f, sizeof(f));
    assert(!AuvH30Frame_Push(&p, 0x59, 0));
    /* Repeated first header byte must still lock onto the following frame. */
    for (i=0; i<sizeof(f); ++i)
        assert(AuvH30Frame_Push(&p,f[i],1) == (i==sizeof(f)-1U));
    p.count=0;
    f[7]^=1U;
    for (i=0; i<sizeof(f); ++i) assert(!AuvH30Frame_Push(&p,f[i],2));
    assert(p.checksum_errors==1U);
    f[7]^=1U;
    for (i=0; i<sizeof(f); ++i)
        assert(AuvH30Frame_Push(&p,f[i],3) == (i==sizeof(f)-1U));
    p.count=0;
    for (i=0; i<5; ++i) assert(!AuvH30Frame_Push(&p,f[i],4));
    for (i=0; i<sizeof(f); ++i)
        assert(AuvH30Frame_Push(&p,f[i],30) == (i==sizeof(f)-1U));
    assert(p.timeouts==1U);
    return 0;
}
