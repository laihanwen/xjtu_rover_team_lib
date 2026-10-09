#ifndef AUV_MODE_H
#define AUV_MODE_H
#include "AuvRovConfig.h"
#include "AuvSafety.h"
#define AUV_HAS_AUTONOMY (AUV_AUTONOMOUS_PROFILE || AUV_DUAL_PROFILE)
#define AUV_HAS_ROV (AUV_ROV_MANUAL_TRIAL || AUV_DUAL_PROFILE)
#define AUV_MODE_ROV 0U
#define AUV_MODE_AUV 1U
#if AUV_DUAL_PROFILE
uint8_t AuvMode_Get(void);
AuvArmResult AuvMode_Select(uint32_t sequence, uint8_t mode, uint8_t armed, uint32_t now_ms);
uint8_t AuvMode_ArmReady(uint32_t now_ms);
#define AUV_MODE_IS_AUV (AuvMode_Get() == AUV_MODE_AUV)
#define AUV_MODE_IS_ROV (AuvMode_Get() == AUV_MODE_ROV)
#else
#define AUV_MODE_IS_AUV AUV_AUTONOMOUS_PROFILE
#define AUV_MODE_IS_ROV AUV_ROV_MANUAL_TRIAL
#endif
#endif
