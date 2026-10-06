#ifndef AUV_ROV_CONFIG_H
#define AUV_ROV_CONFIG_H

/* Explicit manual commissioning profile; autonomous targets remain disabled. */
#ifndef AUV_ROV_MANUAL_TRIAL
#define AUV_ROV_MANUAL_TRIAL 1
#endif
#define AUV_ROV_TRIAL_PWM_LIMIT 100.0f
#define AUV_ROV_HIGH_PWM_LIMIT 125.0f
#define AUV_ROV_TRIAL_ATTITUDE_ENABLED 1
#define AUV_ROV_THRUSTER_CALIBRATION 0
#define AUV_ROV_START_OFFSET_US 48.0f
/* Centered-stick logs: negative yaw correction produced positive yaw rate.
 * Reverse closed-loop feedback only; manual turning signs remain unchanged. */
#define AUV_ROV_YAW_FEEDBACK_SIGN (-1.0f)
/* Zero disables the fixed run-duration limit; failsafe timeouts remain active. */
#define AUV_ROV_TRIAL_ARM_MAX_MS 0U

/* Manual profile: UART2 Pi CRC, UART3 M10. Legacy profile: UART3 Pi,
 * UART2 RC or optional M10, never both. */
#ifndef AUV_UART2_M10_ENABLED
#define AUV_UART2_M10_ENABLED 0
#endif
/* Listen to the field-configured H30; legacy output bitmap is unverified. */
#ifndef AUV_H30_CONFIGURE_ON_BOOT
#define AUV_H30_CONFIGURE_ON_BOOT 0
#endif
/* Enable only after depth rate, vertical direction and gains are measured. */
#ifndef AUV_DEPTH_CONTROL_CALIBRATED
#define AUV_DEPTH_CONTROL_CALIBRATED 0
#endif
#define AUV_DEPTH_KP 300.0f
/* Relative M10 hold candidates, deliberately no integral/derivative on
 * 0.5..1 Hz data. These gains are not tank-calibrated thrust measurements. */
#define AUV_ROV_RELATIVE_DEPTH_ENABLED 1
/* 0 restores manual translation + proven roll/pitch leveling only.
 * 1 replaces SA/SD gating with recentered-axis automatic hold. */
#define AUV_ROV_AUTO_HOLD_ENABLED 1
#define AUV_ROV_DEPTH_KP 40.0f
#define AUV_ROV_DEPTH_OUTPUT_LIMIT 30.0f
#define AUV_DEPTH_KI 1.0f /* output / (metre * second), NOT legacy per-tick KI */
#define AUV_DEPTH_OUTPUT_LIMIT 400.0f
#define AUV_DEPTH_FZ_SIGN (-1.0f) /* depth positive down, body Z positive up */

#endif

