/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define can hfdcan1
#define step_timer htim2
#define tmc_uart huart2
#define encoder_adc hadc1
#define encoder_i2c hi2c1
#define debug_uart huart1
#define ms_timer htim6
#define encoder_Pin GPIO_PIN_0
#define encoder_GPIO_Port GPIOA
#define tmcTX_Pin GPIO_PIN_2
#define tmcTX_GPIO_Port GPIOA
#define tmcRX_Pin GPIO_PIN_3
#define tmcRX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_7
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_0
#define S2_GPIO_Port GPIOB
#define S3_Pin GPIO_PIN_2
#define S3_GPIO_Port GPIOB
#define S4_Pin GPIO_PIN_10
#define S4_GPIO_Port GPIOB
#define DebugTX_Pin GPIO_PIN_9
#define DebugTX_GPIO_Port GPIOA
#define DebugRX_Pin GPIO_PIN_10
#define DebugRX_GPIO_Port GPIOA
#define step_Pin GPIO_PIN_15
#define step_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_1
#define DIR_GPIO_Port GPIOD
#define Diagnose_Pin GPIO_PIN_2
#define Diagnose_GPIO_Port GPIOD
#define Diagnose_EXTI_IRQn EXTI2_3_IRQn
#define MS2_Pin GPIO_PIN_3
#define MS2_GPIO_Port GPIOD
#define MS1_Pin GPIO_PIN_3
#define MS1_GPIO_Port GPIOB
#define Driver_disable_Pin GPIO_PIN_4
#define Driver_disable_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
