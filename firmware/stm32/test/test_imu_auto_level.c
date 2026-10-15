#include "imu.h"
#include <assert.h>
#include <string.h>

uint32_t test_tick, test_primask;
UART_HandleTypeDef huart1;

static void sample(int32_t pitch, int32_t roll)
{
    uint8_t frame[21] = {0x59, 0x53, 0, 0, 14, 0x40, 12};
    int32_t values[3] = {pitch, roll, 30000000};
    uint8_t a = 0, b = 0;
    memcpy(frame + 7, values, 12);
    for (unsigned i = 2; i < 19; ++i) { a = (uint8_t)(a + frame[i]); b = (uint8_t)(b + a); }
    frame[19] = a;
    frame[20] = b;
    h30_parse_data(frame, sizeof(frame));
}

static void stable_samples(uint32_t start)
{
    for (unsigned i = 0; i < 18; ++i) {
        test_tick = start + 120U * i;
        sample(1000000 + (int32_t)(i % 2U) * 100000, -1000000);
        (void)imu_auto_level_tick(1U);
    }
}

int main(void)
{
    test_tick = 100U;
    assert(!imu_auto_level_tick(1U)); /* A boot cannot zero absent IMU. */
    sample(1000000, -1000000);
    assert(!imu_auto_level_tick(0U)); /* No sampling while armed. */
    test_tick = 400U;
    assert(!imu_auto_level_tick(1U)); /* Stale sample cannot count. */
    stable_samples(500U);
    assert(imu_level_calibrated);
    assert(Angle_Measure.pit == 0.0f && Angle_Measure.rol == 0.0f);

    imu_reset_offset();
    test_tick = 5000U;
    sample(10000000, 0); /* Stable but far from upright. */
    assert(!imu_auto_level_tick(1U));
    stable_samples(5200U);
    assert(imu_level_calibrated);

    imu_reset_offset();
    test_tick = 10000U;
    sample(1000000, 0);
    assert(!imu_auto_level_tick(1U));
    test_tick = 10120U;
    sample(3000000, 0); /* Movement resets the stationary interval. */
    assert(!imu_auto_level_tick(1U));
    stable_samples(10240U);
    assert(imu_level_calibrated);

    imu_reset_offset();
    test_tick = 30001U;
    sample(1000000, 0);
    assert(!imu_auto_level_tick(1U));
    assert(!imu_level_calibrated); /* A late reference must be explicit. */
    assert(imu_calibrate_level());
    return 0;
}
