/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
#include "stepperController.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Y_DIR_Pin|Y_EN_Pin|LED_RED_Pin|LED_GRN_Pin
                          |LED_YEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, X_DIR_Pin|X_EN_Pin|LED_BLU_Pin|Z_DIR_Pin
                          |Z_EN_Pin|PUMP_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = Y_DIR_Pin|Y_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = X_DIR_Pin|X_EN_Pin|Z_DIR_Pin|Z_EN_Pin
                          |PUMP_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = Y_LIMIT_Pin|X_LIMIT_Pin|Z_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GRN_Pin|LED_YEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SW_USR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_USR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_BLU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLU_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void led_toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

void led_on(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void led_off(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void setStepperEn(char stepper, uint8_t value)
{
  uint8_t state;

  if (value ==0){
    state = value;
  }else
    state = 1;

  switch (stepper)
  {
  case 'X': HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, state); break;
  case 'Y': HAL_GPIO_WritePin(Y_EN_GPIO_Port, Y_EN_Pin, state); break;
  case 'Z': HAL_GPIO_WritePin(Z_EN_GPIO_Port, Z_EN_Pin, state); break;
  default:
    break;
  }
}

void setPumpEn(uint8_t value)
{
  if (value == 0){
    HAL_GPIO_WritePin(PUMP_SW_GPIO_Port, PUMP_SW_Pin, GPIO_PIN_RESET);
  }else
    HAL_GPIO_WritePin(PUMP_SW_GPIO_Port, PUMP_SW_Pin, GPIO_PIN_SET);
}

void input_scan(void){
  static uint8_t x_limitLastState = 1;
  static uint8_t y_limitLastState = 1;
  static uint8_t z_limitLastState = 1;

  int32_t currentPostion;
  int32_t targetPostion;

  uint8_t x_limitCurrentState = HAL_GPIO_ReadPin(X_LIMIT_GPIO_Port, X_LIMIT_Pin);
  uint8_t y_limitCurrentState = HAL_GPIO_ReadPin(Y_LIMIT_GPIO_Port, Y_LIMIT_Pin);
  uint8_t z_limitCurrentState = HAL_GPIO_ReadPin(Z_LIMIT_GPIO_Port, Z_LIMIT_Pin);
  
  if (x_limitCurrentState != x_limitLastState){ // 边沿触发
    x_limitLastState = x_limitCurrentState;
    if (x_limitCurrentState == GPIO_PIN_SET){   // 高电平表示上升沿
      // 设置目标位置为当前位置，使电机停�?
      targetPostion = Stepper_GetTargetPosition('X');
      currentPostion = Stepper_GetCurrentPosition('X');

      if (targetPostion != currentPostion){
        Stepper_SetTargetPosition('X', Stepper_GetCurrentPosition('X'));
      }
      kprintf("X limit switch close, stop.\r\n");
    }
  }

  if (y_limitCurrentState != y_limitLastState){ // 边沿触发
    y_limitLastState = y_limitCurrentState;
    if (y_limitCurrentState == GPIO_PIN_SET){   // 高电平表示上升沿
      // 设置目标位置为当前位置，使电机停�?
      targetPostion = Stepper_GetTargetPosition('Y');
      currentPostion = Stepper_GetCurrentPosition('Y');

      if (targetPostion != currentPostion){
        Stepper_SetTargetPosition('Y', Stepper_GetCurrentPosition('Y'));
      }
      kprintf("Y limit switch close, stop.\r\n");
    }
  }

  if (z_limitCurrentState != z_limitLastState){ // 边沿触发
    z_limitLastState = z_limitCurrentState;
    if (z_limitCurrentState == GPIO_PIN_SET){   // 高电平表示上升沿
      // 设置目标位置为当前位置，使电机停�?
      targetPostion = Stepper_GetTargetPosition('Z');
      currentPostion = Stepper_GetCurrentPosition('Z');

      if (targetPostion != currentPostion){
        Stepper_SetTargetPosition('Z', Stepper_GetCurrentPosition('Z'));
      }
      kprintf("Z limit switch close, stop.\r\n");
    }
  }
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
