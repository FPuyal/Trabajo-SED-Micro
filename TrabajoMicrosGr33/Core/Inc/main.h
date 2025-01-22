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
#define Trigger_1_USS_Puerta_Pin GPIO_PIN_0
#define Trigger_1_USS_Puerta_GPIO_Port GPIOC
#define Echo_1_USS_Puerta_Pin GPIO_PIN_1
#define Echo_1_USS_Puerta_GPIO_Port GPIOC
#define Trigger_2_USS_Interior_Pin GPIO_PIN_2
#define Trigger_2_USS_Interior_GPIO_Port GPIOC
#define Echo_2_USS_Interior_Pin GPIO_PIN_3
#define Echo_2_USS_Interior_GPIO_Port GPIOC
#define Motor_Pin_1_Pin GPIO_PIN_0
#define Motor_Pin_1_GPIO_Port GPIOD
#define Motor_Pin_2_Pin GPIO_PIN_1
#define Motor_Pin_2_GPIO_Port GPIOD
#define Fin_de_carrera_1_Pin GPIO_PIN_2
#define Fin_de_carrera_1_GPIO_Port GPIOD
#define Fin_de_carrera_2_Pin GPIO_PIN_3
#define Fin_de_carrera_2_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_4
#define LED_R_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOD
#define RED_B_Pin GPIO_PIN_6
#define RED_B_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
