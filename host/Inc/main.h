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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO0_Pin GPIO_PIN_13
#define GPIO0_GPIO_Port GPIOC
#define GPIO1_Pin GPIO_PIN_14
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_15
#define GPIO2_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_0
#define GPIO3_GPIO_Port GPIOA
#define GPIO4_Pin GPIO_PIN_1
#define GPIO4_GPIO_Port GPIOA
#define GPIO5_Pin GPIO_PIN_2
#define GPIO5_GPIO_Port GPIOA
#define GPIO6_Pin GPIO_PIN_3
#define GPIO6_GPIO_Port GPIOA
#define GPIO7_Pin GPIO_PIN_0
#define GPIO7_GPIO_Port GPIOB
#define GPIO8_Pin GPIO_PIN_1
#define GPIO8_GPIO_Port GPIOB
#define GPIO9_Pin GPIO_PIN_2
#define GPIO9_GPIO_Port GPIOB
#define GPIO10_Pin GPIO_PIN_10
#define GPIO10_GPIO_Port GPIOB
#define GPIO11_Pin GPIO_PIN_8
#define GPIO11_GPIO_Port GPIOA
#define GPIO12_Pin GPIO_PIN_9
#define GPIO12_GPIO_Port GPIOA
#define GPIO13_Pin GPIO_PIN_10
#define GPIO13_GPIO_Port GPIOA
#define GPIO14_Pin GPIO_PIN_15
#define GPIO14_GPIO_Port GPIOA
#define GPIO15_Pin GPIO_PIN_3
#define GPIO15_GPIO_Port GPIOB
#define GPIO16_Pin GPIO_PIN_4
#define GPIO16_GPIO_Port GPIOB
#define GPIO17_Pin GPIO_PIN_5
#define GPIO17_GPIO_Port GPIOB
#define GPIO18_Pin GPIO_PIN_8
#define GPIO18_GPIO_Port GPIOB
#define GPIO19_Pin GPIO_PIN_9
#define GPIO19_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
