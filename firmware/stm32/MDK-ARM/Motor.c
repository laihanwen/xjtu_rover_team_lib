#include "motor.h"
#include "main.h"
#include "stm32f4xx_hal.h"

uint16_t motor_outputs[MOTOR_COUNT];
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;
static uint16_t constrain_pwm(uint16_t pwm) {
    if (pwm < 1000) return 1000;
    if (pwm > 2000) return 2000;
    return pwm;
}


