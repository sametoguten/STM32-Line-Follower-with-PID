/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define SENSOR8_Pin GPIO_PIN_0
#define SENSOR8_GPIO_Port GPIOA
#define SENSOR7_Pin GPIO_PIN_1
#define SENSOR7_GPIO_Port GPIOA
#define SENSOR6_Pin GPIO_PIN_2
#define SENSOR6_GPIO_Port GPIOA
#define SENSOR5_Pin GPIO_PIN_3
#define SENSOR5_GPIO_Port GPIOA
#define SENSOR4_Pin GPIO_PIN_4
#define SENSOR4_GPIO_Port GPIOA
#define SENSOR3_Pin GPIO_PIN_5
#define SENSOR3_GPIO_Port GPIOA
#define SENSOR2_Pin GPIO_PIN_6
#define SENSOR2_GPIO_Port GPIOA
#define SENSOR1_Pin GPIO_PIN_7
#define SENSOR1_GPIO_Port GPIOA
#define LEDON_Pin GPIO_PIN_0
#define LEDON_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
