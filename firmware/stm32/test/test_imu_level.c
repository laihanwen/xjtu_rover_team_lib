#include "imu.h"
#include <assert.h>
#include <string.h>
uint32_t test_tick, test_primask;
UART_HandleTypeDef huart1;
static void sample(int32_t pitch,int32_t roll,int32_t yaw)
{
    uint8_t frame[21]={0x59,0x53,0,0,14,0x40,12};
    int32_t values[3]={pitch,roll,yaw};
    uint8_t a=0,b=0;
    memcpy(frame+7,values,12);
    for (unsigned i=2;i<19;++i) { a=(uint8_t)(a+frame[i]); b=(uint8_t)(b+a); }
    frame[19]=a; frame[20]=b;
    h30_parse_data(frame,sizeof(frame));
}
int main(void)
{
    FLOAT_Angle angle;
    uint32_t seq,stamp;
    test_tick=100;
    assert(!imu_calibrate_level());
    sample(10000000,-5000000,30000000);
    assert(!imu_level_calibrated);
    assert(Angle_Measure.pit==10 && Angle_Measure.rol==-5);
    assert(imu_calibrate_level());
    assert(imu_level_calibrated);
    test_tick=110;
    sample(12000000,-2000000,31000000);
    assert(Angle_Measure.pit==2 && Angle_Measure.rol==3);
    test_primask=1;
    assert(imu_copy_fresh(&angle,&seq,&stamp));
    assert(test_primask==1 && stamp==110 && seq==2);
    test_primask=0;
    test_tick=361;
    assert(!imu_copy_fresh(&angle,&seq,&stamp));
    assert(test_primask==0 && !imu_calibrate_level());
    test_tick=0xfffffff0;
    sample(14000000,179000000,32000000);
    assert(imu_calibrate_level());
    test_tick=0x10;
    sample(15000000,-179000000,33000000);
    assert(imu_copy_fresh(&angle,&seq,&stamp));
    assert(angle.pit==1 && angle.rol==2);
    imu_reset_offset();
    assert(!imu_level_calibrated);
    return 0;
}
