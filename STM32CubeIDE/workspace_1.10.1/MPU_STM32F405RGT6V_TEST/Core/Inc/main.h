/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define FAULT_MCU_Pin GPIO_PIN_3
#define FAULT_MCU_GPIO_Port GPIOC
#define BSE_1_Pin GPIO_PIN_0
#define BSE_1_GPIO_Port GPIOA
#define BSE_2_Pin GPIO_PIN_1
#define BSE_2_GPIO_Port GPIOA
#define APPS_1_Pin GPIO_PIN_2
#define APPS_1_GPIO_Port GPIOA
#define APPS_2_Pin GPIO_PIN_3
#define APPS_2_GPIO_Port GPIOA
#define GPIO_1_Pin GPIO_PIN_4
#define GPIO_1_GPIO_Port GPIOC
#define GPIO_2_Pin GPIO_PIN_5
#define GPIO_2_GPIO_Port GPIOC
#define GPIO_3_Pin GPIO_PIN_0
#define GPIO_3_GPIO_Port GPIOB
#define GPIO_4_Pin GPIO_PIN_1
#define GPIO_4_GPIO_Port GPIOB
#define WATCHDOG_Pin GPIO_PIN_15
#define WATCHDOG_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_8
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_9
#define LED_2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
