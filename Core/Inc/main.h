/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "common.h"
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
#define UART_TX_Pin GPIO_PIN_2
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_3
#define UART_RX_GPIO_Port GPIOA
#define M1_DIR_Pin GPIO_PIN_4
#define M1_DIR_GPIO_Port GPIOA
#define M1_STEP_Pin GPIO_PIN_5
#define M1_STEP_GPIO_Port GPIOA
#define M3_STEP_Pin GPIO_PIN_6
#define M3_STEP_GPIO_Port GPIOA
#define M0_STEP_Pin GPIO_PIN_7
#define M0_STEP_GPIO_Port GPIOA
#define PUMP_SW_Pin GPIO_PIN_5
#define PUMP_SW_GPIO_Port GPIOC
#define M0_DIR_Pin GPIO_PIN_10
#define M0_DIR_GPIO_Port GPIOB
#define M3_DIR_Pin GPIO_PIN_11
#define M3_DIR_GPIO_Port GPIOB
#define M1_LIMIT_Pin GPIO_PIN_12
#define M1_LIMIT_GPIO_Port GPIOB
#define M2_STEP_Pin GPIO_PIN_13
#define M2_STEP_GPIO_Port GPIOB
#define M0_LIMIT_Pin GPIO_PIN_14
#define M0_LIMIT_GPIO_Port GPIOB
#define M2_LIMIT_Pin GPIO_PIN_15
#define M2_LIMIT_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOA
#define LED_GRN_Pin GPIO_PIN_14
#define LED_GRN_GPIO_Port GPIOA
#define LED_YEL_Pin GPIO_PIN_15
#define LED_YEL_GPIO_Port GPIOA
#define SW_USR_Pin GPIO_PIN_3
#define SW_USR_GPIO_Port GPIOB
#define LED_BLU_Pin GPIO_PIN_4
#define LED_BLU_GPIO_Port GPIOB
#define M2_DIR_Pin GPIO_PIN_6
#define M2_DIR_GPIO_Port GPIOB
#define M3_LIMIT_Pin GPIO_PIN_7
#define M3_LIMIT_GPIO_Port GPIOB
#define M4_ANGLE_Pin GPIO_PIN_8
#define M4_ANGLE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
