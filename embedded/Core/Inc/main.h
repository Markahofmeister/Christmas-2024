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
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define LED10_Pin GPIO_PIN_10
#define LED10_GPIO_Port GPIOB
#define LED11_Pin GPIO_PIN_11
#define LED11_GPIO_Port GPIOB
#define LED12_Pin GPIO_PIN_12
#define LED12_GPIO_Port GPIOB
#define LED13_Pin GPIO_PIN_13
#define LED13_GPIO_Port GPIOB
#define LED14_Pin GPIO_PIN_14
#define LED14_GPIO_Port GPIOB
#define LED15_Pin GPIO_PIN_15
#define LED15_GPIO_Port GPIOB
#define PWM_0_Pin GPIO_PIN_8
#define PWM_0_GPIO_Port GPIOA
#define PWM_1_Pin GPIO_PIN_9
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_6
#define PWM_2_GPIO_Port GPIOC
#define PWM_3_Pin GPIO_PIN_7
#define PWM_3_GPIO_Port GPIOC
#define AMP_ENABLE_Pin GPIO_PIN_0
#define AMP_ENABLE_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_7
#define LED7_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_8
#define LED8_GPIO_Port GPIOB
#define LED9_Pin GPIO_PIN_9
#define LED9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
