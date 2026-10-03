/**
 * @file AuvGripperConfig.h
 * @brief T35-L single-servo gripper calibration and safety configuration.
 */

#ifndef AUV_GRIPPER_CONFIG_H
#define AUV_GRIPPER_CONFIG_H

/*
 * Keep this gate at zero until OPEN/CLOSE pulse widths have been measured on
 * the unloaded mechanism. With the gate disabled, commands are rejected and
 * PA8/TIM1_CH1 stays at the neutral pulse.
 */
#define AUV_GRIPPER_CALIBRATED          0U

#define AUV_GRIPPER_MIN_US              500U
#define AUV_GRIPPER_NEUTRAL_US         1500U
#define AUV_GRIPPER_MAX_US             2500U

/* T35-L direction confirmed by the mechanism: larger pulse opens the jaw. */
#define AUV_GRIPPER_CLOSE_US            1000U /* TODO: replace with measured value. */
#define AUV_GRIPPER_OPEN_US             2000U /* TODO: replace with measured value. */

/* Mate_Task runs every 10 ms. 10 us/tick gives a deliberately gentle ramp. */
#define AUV_GRIPPER_SLEW_US_PER_TICK      10U

/* Existing remote mapping: SB selects/enables the gripper, SA requests state. */
#define AUV_GRIPPER_RC_CLOSE_MAX          80U
#define AUV_GRIPPER_RC_OPEN_MIN          175U

#endif
