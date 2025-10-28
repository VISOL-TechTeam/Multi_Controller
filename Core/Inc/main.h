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
extern "C"
{
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

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

  /* USER CODE BEGIN EFP */
  void RTOS_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOC
#define Trigger_IN_1_Pin GPIO_PIN_1
#define Trigger_IN_1_GPIO_Port GPIOA
#define Trigger_IN_2_Pin GPIO_PIN_2
#define Trigger_IN_2_GPIO_Port GPIOA
#define Trigger_OUT_1_Pin GPIO_PIN_3
#define Trigger_OUT_1_GPIO_Port GPIOA
#define Trigger_OUT_2_Pin GPIO_PIN_4
#define Trigger_OUT_2_GPIO_Port GPIOA
#define Dial_LED_1_Pin GPIO_PIN_5
#define Dial_LED_1_GPIO_Port GPIOA
#define Dial_LED_2_Pin GPIO_PIN_6
#define Dial_LED_2_GPIO_Port GPIOA
#define Dial_LED_3_Pin GPIO_PIN_7
#define Dial_LED_3_GPIO_Port GPIOA
#define Dial_A_Pin GPIO_PIN_0
#define Dial_A_GPIO_Port GPIOB
#define Dial_B_Pin GPIO_PIN_1
#define Dial_B_GPIO_Port GPIOB
#define RIGHT_Pin GPIO_PIN_10
#define RIGHT_GPIO_Port GPIOB
#define BOOST_Pin GPIO_PIN_12
#define BOOST_GPIO_Port GPIOB
#define POWRAY_ON_Pin GPIO_PIN_13
#define POWRAY_ON_GPIO_Port GPIOB
#define DOWN_Pin GPIO_PIN_14
#define DOWN_GPIO_Port GPIOB
#define UP_Pin GPIO_PIN_15
#define UP_GPIO_Port GPIOB
#define System_LED_Pin GPIO_PIN_6
#define System_LED_GPIO_Port GPIOC
#define USB_SW_Pin GPIO_PIN_11
#define USB_SW_GPIO_Port GPIOC
#define Memory_2_Pin GPIO_PIN_3
#define Memory_2_GPIO_Port GPIOB
#define Memory_3_Pin GPIO_PIN_4
#define Memory_3_GPIO_Port GPIOB
#define Memory_4_Pin GPIO_PIN_5
#define Memory_4_GPIO_Port GPIOB
#define Memory_1_Pin GPIO_PIN_2
#define Memory_1_GPIO_Port GPIOD
#define MODE_Pin GPIO_PIN_6
#define MODE_GPIO_Port GPIOB
#define Power_SW_Pin GPIO_PIN_8
#define Power_SW_GPIO_Port GPIOB
#define LEFT_Pin GPIO_PIN_9
#define LEFT_GPIO_Port GPIOB

  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
