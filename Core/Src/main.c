/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stepperController.h"
#include "stepperCommands.h"
#include "serial.h"

#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define X_TIM     htim8
#define X_TIM_CH  TIM_CHANNEL_1
#define Y_TIM     htim2
#define Y_TIM_CH  TIM_CHANNEL_1
#define Z_TIM     htim1
#define Z_TIM_CH  TIM_CHANNEL_1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// #define TEST
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t STEP_TIMER_CLOCK;
uint32_t STEP_CONTROLLER_PERIOD_US;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  stepper_request stReq =  {'\0', CMD_GET, PARAM_ALL, 0, false};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  STEP_TIMER_CLOCK = HAL_RCC_GetHCLKFreq() / 2;
  STEP_CONTROLLER_PERIOD_US =  1000000U /(STEP_TIMER_CLOCK / htim5.Init.Period);
  
  HAL_Delay(1);

  kprintf("\r\n");
  kprintf ("============== Stepper Hub ================\r\n");
  kprintf ("  Timer Clock: %ld MHz StepperCtrl: %ld us\r\n", STEP_TIMER_CLOCK/1000000, STEP_CONTROLLER_PERIOD_US);
  kprintf ("  X Step:PA7  Tim:8 Ch:1 Dir: PB10 En: none\r\n");
  kprintf ("  Y Step:PA5  Tim:2 Ch:1 Dir: PA4  En: none\r\n");
  kprintf ("  Z Step:PB13 Tim:1 Ch:1 Dir: PB12 En: none\r\n");
  kprintf ("===========================================\r\n");
  kprintf("\r\n");

  Stepper_SetupPeripherals('X', &X_TIM, X_TIM_CH, &HAL_TIMEx_PWMN_Start, &HAL_TIMEx_PWMN_Stop, X_DIR_GPIO_Port, X_DIR_Pin);
  Stepper_SetupPeripherals('Y', &Y_TIM, Y_TIM_CH, &HAL_TIM_PWM_Start, &HAL_TIM_PWM_Stop, Y_DIR_GPIO_Port, Y_DIR_Pin);
  Stepper_SetupPeripherals('Z', &Z_TIM, Z_TIM_CH, &HAL_TIMEx_PWMN_Start, &HAL_TIMEx_PWMN_Stop, Z_DIR_GPIO_Port, Z_DIR_Pin);
  
  kprintf("Reading settings from internal storage...\r\n");
  Stepper_LoadConfig();

  if (Stepper_GetAccPrescaler('Z') == 0xFFFFFFFF) {
    kprintf("Storage is clean, initializing defaults ...\r\n");
    Stepper_InitDefaultState('X');
    Stepper_InitDefaultState('Y');
    Stepper_InitDefaultState('Z');
    Stepper_SaveConfig();
  } 
  kprintf("DONE!\r\n\r\n");

  __HAL_TIM_ENABLE_IT(&X_TIM, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&Y_TIM, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&Z_TIM, TIM_IT_UPDATE);

  Serial_InitRxSequence();

  HAL_Delay(1);
  // This will run our StepController timer and enable interrupt for it as well
  HAL_TIM_Base_Start_IT(&htim5);
  
  // TODO: load settingsfrom FLASH
  stReq.stepper = 'X';
  ExecuteRequest(&stReq);
  stReq.stepper = 'Y';
  ExecuteRequest(&stReq);
  stReq.stepper = 'Z';
  ExecuteRequest(&stReq);

#if defined (TEST) 

  Stepper_SetTargetPosition('X',100);

#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  usb_printf("error\r\n");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: kprintf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
