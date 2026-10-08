/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "RC.h"
#include "imu.h"

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include "AuvLink.h"
#include "AuvRcInput.h"
#include "AuvRovConfig.h"
#include "AuvRovDepth.h"
static AuvM10Parser m10_parser;
static uint8_t usart3_depth_byte;
// 定义正确的帧长度（根据协议最大262字节）
#define IMU229_MAX_FRAME_LEN 262

uint8_t usart1_rx_buf[IMU229_MAX_FRAME_LEN] = {0};
uint16_t usart1_rx_cnt = 0;
uint8_t usart2_rx_byte = 0;

//**回调函数**//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint8_t received_byte = usart1_rx_buf[0];
        /* Re-arm before checksum/Euler work at 460800 baud. */
        if (HAL_UART_Receive_IT(&huart1, &usart1_rx_buf[0], 1U) != HAL_OK)
            imu_rx_restart_failures++;
        h30_data_callback(received_byte);
    }
		if (huart->Instance == USART2){
#if AUV_CURRENT_UART_LAYOUT
            AuvLink_RxComplete(huart);
#else
			#if AUV_UART2_M10_ENABLED
            float depth;
            int result = AuvM10_Push(&m10_parser, usart2_rx_byte, &depth);
            if (result == 1) (void)AuvLink_UpdateDepth(depth);
            else if (result < 0) AuvLink_InvalidateDepth();
#else
            AuvRcInput_PushByte(usart2_rx_byte, HAL_GetTick());
#endif
		    HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1);
#endif
	  }
		if (huart->Instance == USART3){
#if AUV_CURRENT_UART_LAYOUT
            uint8_t byte = usart3_depth_byte;
            float depth;
            int result;
            (void)HAL_UART_Receive_IT(&huart3, &usart3_depth_byte, 1U);
            result = AuvM10_Push(&m10_parser, byte, &depth);
            if (result == 1) (void)AuvLink_UpdateDepth(depth);
            else if (result < 0) AuvLink_InvalidateDepth();
#else
            AuvLink_RxComplete(huart);
#endif
		}
}

// 初始化接收
void USART1_Receive_IT_Init(void) {
    usart1_rx_cnt = 0;
    if (HAL_UART_Receive_IT(&huart1, &usart1_rx_buf[0], 1U) != HAL_OK)
        imu_rx_restart_failures++;
}
void USART2_Receive_IT_Init(void) {
  AuvRcInput_Init();
  AuvM10_Reset(&m10_parser);
#if AUV_CURRENT_UART_LAYOUT
  huart3.Init.BaudRate = 115200U;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
  (void)HAL_UART_Receive_IT(&huart3, &usart3_depth_byte, 1U);
#else
  HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1);
#endif
}

void USART1_SetBaudRate_460800(void) {
	  HAL_UART_DeInit(&huart1);
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 460800;  // H30默认波特率
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
	if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->ErrorCode & HAL_UART_ERROR_ORE)
        __HAL_UART_CLEAR_OREFLAG(huart);
    if (huart->Instance == USART1) {
        imu_uart_errors++;
        h30_reset_rx();
        /* PE/FE/NE may leave reception active; never move its destination. */
        if (huart1.RxState == HAL_UART_STATE_READY &&
            HAL_UART_Receive_IT(&huart1, &usart1_rx_buf[0], 1U) != HAL_OK)
            imu_rx_restart_failures++;
    } else if (huart->Instance == USART2) {
#if AUV_CURRENT_UART_LAYOUT
        AuvLink_RxError(huart);
#else
        AuvRcInput_Init();
        AuvM10_Reset(&m10_parser);
#if AUV_UART2_M10_ENABLED
        AuvLink_InvalidateDepth();
#endif
        (void)HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U);
#endif
    } else {
#if AUV_CURRENT_UART_LAYOUT
        AuvM10_Reset(&m10_parser);
        AuvLink_InvalidateDepth();
        if (huart3.RxState == HAL_UART_STATE_READY)
            (void)HAL_UART_Receive_IT(&huart3,&usart3_depth_byte,1U);
#else
        AuvLink_RxError(huart);
#endif
    }
}
void USART1_Receive_Service(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (huart1.RxState == HAL_UART_STATE_READY) {
        h30_reset_rx();
        if (HAL_UART_Receive_IT(&huart1, &usart1_rx_buf[0], 1U) != HAL_OK)
            imu_rx_restart_failures++;
    }
    __set_PRIMASK(primask);
}
/* USER CODE END 1 */
