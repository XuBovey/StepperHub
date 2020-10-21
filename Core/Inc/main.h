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
#define Y_DIR_Pin GPIO_PIN_4
#define Y_DIR_GPIO_Port GPIOA
#define Y_STEP_Pin GPIO_PIN_5
#define Y_STEP_GPIO_Port GPIOA
#define Y_EN_Pin GPIO_PIN_6
#define Y_EN_GPIO_Port GPIOA
#define X_DIR_Pin GPIO_PIN_10
#define X_DIR_GPIO_Port GPIOB
#define X_EN_Pin GPIO_PIN_11
#define X_EN_GPIO_Port GPIOB
#define Z_DIR_Pin GPIO_PIN_12
#define Z_DIR_GPIO_Port GPIOB
#define Z_STEP_Pin GPIO_PIN_13
#define Z_STEP_GPIO_Port GPIOB
#define Z_EN_Pin GPIO_PIN_14
#define Z_EN_GPIO_Port GPIOB
#define Z_LIMIT_Pin GPIO_PIN_15
#define Z_LIMIT_GPIO_Port GPIOB
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
#define Y_LIMIT_Pin GPIO_PIN_6
#define Y_LIMIT_GPIO_Port GPIOB
#define X_LIMIT_Pin GPIO_PIN_7
#define X_LIMIT_GPIO_Port GPIOB
#define PUMP_SW_Pin GPIO_PIN_8
#define PUMP_SW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
