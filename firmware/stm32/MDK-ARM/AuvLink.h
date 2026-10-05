/**
 * @file AuvLink.h
 * @brief Raspberry Pi UART link and safety integration.
 */

#ifndef AUV_LINK_H
#define AUV_LINK_H

#include "main.h"

void AuvLink_Init(void);
void AuvLink_Task(void);
void AuvLink_RxComplete(UART_HandleTypeDef *huart);
void AuvLink_SetSafetyInputs(uint8_t leak_detected,
                             uint8_t kill_active,
                             uint8_t sensors_valid);
uint8_t AuvLink_UpdateDepth(float depth_m);
void AuvLink_InvalidateDepth(void);

#endif
