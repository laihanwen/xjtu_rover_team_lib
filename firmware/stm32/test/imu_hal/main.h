#ifndef TEST_IMU_HAL_H
#define TEST_IMU_HAL_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
extern uint32_t test_tick, test_primask;
static inline uint32_t HAL_GetTick(void) { return test_tick; }
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask=1; }
static inline void __set_PRIMASK(uint32_t value) { test_primask=value; }
static inline void HAL_Delay(uint32_t delay) { test_tick+=delay; }
static inline int HAL_UART_Transmit(UART_HandleTypeDef *uart,uint8_t *bytes,uint16_t length,uint32_t timeout)
{ (void)uart; (void)bytes; (void)length; (void)timeout; return 0; }
#endif
