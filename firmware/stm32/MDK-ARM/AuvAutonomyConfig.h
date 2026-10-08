#ifndef AUV_AUTONOMY_CONFIG_H
#define AUV_AUTONOMY_CONFIG_H

/* Select only in the independent AUV build. No physical outputs are enabled
 * by selecting this profile. Commission all gates with actual hardware. */
#ifndef AUV_AUTONOMOUS_PROFILE
#define AUV_AUTONOMOUS_PROFILE 0
#endif
#ifndef AUV_AUTONOMY_COMMISSIONED
#define AUV_AUTONOMY_COMMISSIONED 0
#endif
#ifndef AUV_DEPTH_ZERO_CALIBRATED
#define AUV_DEPTH_ZERO_CALIBRATED 0
#endif
#ifndef AUV_DEPTH_ZERO_M
#define AUV_DEPTH_ZERO_M 0.0f /* TODO: measured surface gauge reading */
#endif
/* Same conservative output ceiling as the established ROV low-speed profile. */
#define AUV_AUTONOMY_PWM_LIMIT 100.0f
#ifndef AUV_AUTONOMY_SURGE_PWM_PER_TARGET
#define AUV_AUTONOMY_SURGE_PWM_PER_TARGET 0.0f /* TODO: restrained-tank calibration */
#endif
#ifndef AUV_AUTONOMY_SWAY_PWM_PER_TARGET
#define AUV_AUTONOMY_SWAY_PWM_PER_TARGET 0.0f
#endif
#ifndef AUV_AUTONOMY_DEPTH_FZ_SIGN
#define AUV_AUTONOMY_DEPTH_FZ_SIGN 0.0f /* TODO: +1/-1, measured correction direction */
#endif
#ifndef AUV_AUTONOMY_MAX_DEPTH_M
#define AUV_AUTONOMY_MAX_DEPTH_M 0.0f /* TODO: verified safe pool depth envelope */
#endif
#define AUV_AUTONOMY_READY (AUV_AUTONOMY_COMMISSIONED && AUV_DEPTH_CONTROL_CALIBRATED && \
    AUV_DEPTH_ZERO_CALIBRATED && \
    (AUV_AUTONOMY_DEPTH_FZ_SIGN == 1.0f || AUV_AUTONOMY_DEPTH_FZ_SIGN == -1.0f) && \
    (AUV_AUTONOMY_SURGE_PWM_PER_TARGET > 0.0f) && \
    (AUV_AUTONOMY_SWAY_PWM_PER_TARGET > 0.0f) && \
    (AUV_AUTONOMY_MAX_DEPTH_M > 0.0f && AUV_AUTONOMY_MAX_DEPTH_M <= 100.0f))

#endif
