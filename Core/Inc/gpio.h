/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define K1_State HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)
#define K2_State HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)
#define K3_State HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin)
#define K4_State HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin)

#define LED1_ON HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET)
#define LED2_ON HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET)
#define LED3_ON HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET)
#define LED4_ON HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET)

#define LED1_OFF HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET)
#define LED2_OFF HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET)
#define LED3_OFF HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET)
#define LED4_OFF HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET)

#define LED1_TOGGLE HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin )
#define LED2_TOGGLE HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin )
#define LED3_TOGGLE HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin )
#define LED4_TOGGLE HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin )

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

