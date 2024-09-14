/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

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
#define RGB_Pin GPIO_PIN_1
#define RGB_GPIO_Port GPIOA
#define MODE_SWITCH_Pin GPIO_PIN_2
#define MODE_SWITCH_GPIO_Port GPIOA
#define RELAY1_Pin GPIO_PIN_3
#define RELAY1_GPIO_Port GPIOA
#define RELAY2_Pin GPIO_PIN_4
#define RELAY2_GPIO_Port GPIOA
#define RELAY_TEST_Pin GPIO_PIN_5
#define RELAY_TEST_GPIO_Port GPIOA
#define CUR_Pin GPIO_PIN_6
#define CUR_GPIO_Port GPIOA
#define CUR_FAULT_Pin GPIO_PIN_7
#define CUR_FAULT_GPIO_Port GPIOA
#define HORN_SWITCH_Pin GPIO_PIN_0
#define HORN_SWITCH_GPIO_Port GPIOB
#define BAT_Pin GPIO_PIN_1
#define BAT_GPIO_Port GPIOB
#define START_SWITCH_Pin GPIO_PIN_15
#define START_SWITCH_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOB
#define IO_RST_Pin GPIO_PIN_4
#define IO_RST_GPIO_Port GPIOB
#define RELAY1_LED_Pin GPIO_PIN_5
#define RELAY1_LED_GPIO_Port GPIOB
#define RELAY2_LED_Pin GPIO_PIN_6
#define RELAY2_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
