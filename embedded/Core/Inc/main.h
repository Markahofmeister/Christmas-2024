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
#define SPI_CHIP_SELECT_Pin GPIO_PIN_5
#define SPI_CHIP_SELECT_GPIO_Port GPIOA
#define MEM_nWP_Pin GPIO_PIN_6
#define MEM_nWP_GPIO_Port GPIOA
#define MEM_nHOLD_Pin GPIO_PIN_7
#define MEM_nHOLD_GPIO_Port GPIOA
#define AMP_ENABLE_Pin GPIO_PIN_1
#define AMP_ENABLE_GPIO_Port GPIOB
#define DEBUG_LED_Pin GPIO_PIN_12
#define DEBUG_LED_GPIO_Port GPIOB
#define SHIFT_DATA_IN_Pin GPIO_PIN_9
#define SHIFT_DATA_IN_GPIO_Port GPIOA
#define PWM_SHIFT_nOE_Pin GPIO_PIN_6
#define PWM_SHIFT_nOE_GPIO_Port GPIOC
#define SHIFT_STORE_CLK_Pin GPIO_PIN_7
#define SHIFT_STORE_CLK_GPIO_Port GPIOC
#define SHIFT_DATA_CLK_Pin GPIO_PIN_10
#define SHIFT_DATA_CLK_GPIO_Port GPIOA
#define SHIFT_MCLR_Pin GPIO_PIN_11
#define SHIFT_MCLR_GPIO_Port GPIOA
#define ALARM_SET_BUTTON_EXTI_Pin GPIO_PIN_15
#define ALARM_SET_BUTTON_EXTI_GPIO_Port GPIOA
#define ALARM_SET_BUTTON_EXTI_EXTI_IRQn EXTI4_15_IRQn
#define HOUR_SET_BUTTON_EXTI_Pin GPIO_PIN_0
#define HOUR_SET_BUTTON_EXTI_GPIO_Port GPIOD
#define HOUR_SET_BUTTON_EXTI_EXTI_IRQn EXTI0_1_IRQn
#define MINUTE_SET_BUTTON_EXTI_Pin GPIO_PIN_1
#define MINUTE_SET_BUTTON_EXTI_GPIO_Port GPIOD
#define MINUTE_SET_BUTTON_EXTI_EXTI_IRQn EXTI0_1_IRQn
#define ALARM_EN_BUTTON_EXTI_Pin GPIO_PIN_2
#define ALARM_EN_BUTTON_EXTI_GPIO_Port GPIOD
#define ALARM_EN_BUTTON_EXTI_EXTI_IRQn EXTI2_3_IRQn
#define DISPLAY_BUTTON_EXTI_Pin GPIO_PIN_3
#define DISPLAY_BUTTON_EXTI_GPIO_Port GPIOD
#define DISPLAY_BUTTON_EXTI_EXTI_IRQn EXTI2_3_IRQn
#define TIME_SWITCH_EXTI_Pin GPIO_PIN_4
#define TIME_SWITCH_EXTI_GPIO_Port GPIOB
#define TIME_SWITCH_EXTI_EXTI_IRQn EXTI4_15_IRQn
#define CAPTOUCH_RESET_Pin GPIO_PIN_8
#define CAPTOUCH_RESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
