/**
 * @file AuvCameraServoConfig.h
 * @brief PC7/TIM8_CH2 camera servo calibration gate.
 */

#ifndef AUV_CAMERA_SERVO_CONFIG_H
#define AUV_CAMERA_SERVO_CONFIG_H

/* Installed camera linkage and zero confirmed by operator; no automatic flash. */
#define AUV_CAMERA_SERVO_CALIBRATED       1U

/* Imported from the supplied camera build; verify these CCR values on the real mechanism. */
#define AUV_CAMERA_SERVO_MIN_CCR       2550U /* nominal -45 degrees */
#define AUV_CAMERA_SERVO_MAX_CCR       3100U /* nominal +10 degrees */
#define AUV_CAMERA_SERVO_STARTUP_CCR   3000U /* operator-confirmed zero = 1500 us */
#define AUV_CAMERA_SERVO_SLEW_PER_TICK   10U

#endif
