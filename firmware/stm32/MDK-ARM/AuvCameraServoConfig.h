/**
 * @file AuvCameraServoConfig.h
 * @brief PC7/TIM8_CH2 camera servo calibration gate.
 */

#ifndef AUV_CAMERA_SERVO_CONFIG_H
#define AUV_CAMERA_SERVO_CONFIG_H

/* Keep disabled until the installed linkage has been measured without thruster power. */
#define AUV_CAMERA_SERVO_CALIBRATED       0U

/* Imported from the supplied camera build; verify these CCR values on the real mechanism. */
#define AUV_CAMERA_SERVO_MIN_CCR       2250U /* nominal -75 degrees */
#define AUV_CAMERA_SERVO_MAX_CCR       3000U /* nominal   0 degrees */
#define AUV_CAMERA_SERVO_STARTUP_CCR   3000U /* safest supplied endpoint; no startup sweep */
#define AUV_CAMERA_SERVO_SLEW_PER_TICK   10U

#endif
