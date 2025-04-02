/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
#define Trig_Pin GPIO_PIN_13
#define Trig_GPIO_Port GPIOB
#define Echo_Pin GPIO_PIN_14
#define Echo_GPIO_Port GPIOB
#define LED_Buzzer_Pin GPIO_PIN_7
#define LED_Buzzer_GPIO_Port GPIOC
#define DATA_AVAILABLE_Pin GPIO_PIN_8
#define DATA_AVAILABLE_GPIO_Port GPIOC
#define DATA_AVAILABLE_EXTI_IRQn EXTI9_5_IRQn
#define DATA0_Pin GPIO_PIN_8
#define DATA0_GPIO_Port GPIOA
#define DATA1_Pin GPIO_PIN_9
#define DATA1_GPIO_Port GPIOA
#define DATA2_Pin GPIO_PIN_10
#define DATA2_GPIO_Port GPIOA
#define DATA3_Pin GPIO_PIN_11
#define DATA3_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
