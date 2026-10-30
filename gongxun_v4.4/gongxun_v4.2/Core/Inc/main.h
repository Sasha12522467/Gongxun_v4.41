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
#define START_KEY2_Pin GPIO_PIN_13
#define START_KEY2_GPIO_Port GPIOC
#define START_KEY_Pin GPIO_PIN_12
#define START_KEY_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
/* 两个上拉启动键：PD12选1号区，PD13选2号区；低电平有效。
 * 兼容旧代码的Pin2/Port2命名，正式主流程使用生成的START_KEY2宏。
 */
#ifndef START_KEY_Pin
#define START_KEY_Pin GPIO_PIN_12
#endif
#ifndef START_KEY_GPIO_Port
#define START_KEY_GPIO_Port GPIOD
#endif

#ifndef START_KEY_Pin2
#define START_KEY_Pin2 START_KEY2_Pin
#endif
#ifndef START_KEY_GPIO_Port2
#define START_KEY_GPIO_Port2 START_KEY2_GPIO_Port
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
