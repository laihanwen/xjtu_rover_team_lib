/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define IMU229_FRAME_LEN 262 
extern uint8_t usart1_rx_buf[IMU229_FRAME_LEN];
extern uint16_t usart1_rx_cnt;
extern uint8_t imu_data_ready;  // ???????
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
void USART1_Receive_IT_Init(void);
void USART2_Receive_IT_Init(void);
void USART1_SetBaudRate_460800(void);
uint8_t h30_data_callback(uint8_t byte);
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
 // ???????
extern uint8_t usart1_rx_buf[IMU229_FRAME_LEN];
extern uint16_t usart1_rx_cnt;
extern uint8_t imu_data_ready;  // ???????
extern uint8_t RCflag;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

