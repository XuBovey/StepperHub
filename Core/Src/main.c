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
#ifdef ARM_ROBOT
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
#endif

#ifdef  BELT_ROBOT
extern TIM_HandleTypeDef htim13;
#endif

_moto motoTable[MOTO_MAX] = {
#ifdef ARM_ROBOT
  {'M', &htim8,  TIM_CHANNEL_1}, // M0 X8
  {'N', &htim2,  TIM_CHANNEL_1}, // M1 X6
  {'O', &htim1,  TIM_CHANNEL_1}, // M2 Y6
#endif

#ifdef  BELT_ROBOT
  {'L', &htim13, TIM_CHANNEL_1}, // M3
#endif
};

_moto_limited moto_limited_io[MOTO_MAX] = {
#ifdef ARM_ROBOT
  {1, 1, 0, M0_LIMIT_GPIO_Port, M0_LIMIT_Pin}, //Y7
  {1, 1, 0, M1_LIMIT_GPIO_Port, M1_LIMIT_Pin}, //Y5
  {1, 1, 0, M2_LIMIT_GPIO_Port, M2_LIMIT_Pin}, //Y8
#endif

#ifdef  BELT_ROBOT
  {1, 1, 0, M3_LIMIT_GPIO_Port, M3_LIMIT_Pin}, //X10
#endif
};


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
  int i;
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
  MX_TIM10_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  STEP_TIMER_CLOCK = HAL_RCC_GetHCLKFreq() / 2;
  STEP_CONTROLLER_PERIOD_US =  1000000U /(STEP_TIMER_CLOCK / htim5.Init.Period);
  
  HAL_Delay(1);
#if 1
  kprintf("\r\n");
  kprintf ("========= Stepper Hub for STM32F405 ==========\r\n");
  kprintf ("  Timer Clock: %ld MHz StepperCtrl: %ld us\r\n", STEP_TIMER_CLOCK/1000000, STEP_CONTROLLER_PERIOD_US);
  kprintf ("  M0 X/M Step:PA7  Tim:8 Ch:1N Dir: PB10 En: none\r\n");
  kprintf ("  M1 Y/N Step:PA5  Tim:2 Ch:1  Dir: PA4  En: none\r\n");
  kprintf ("  M2 Z/O Step:PB13 Tim:1 Ch:1N Dir: PB12 En: none\r\n");
  kprintf ("  M3 L Step:PA6 Tim:13 Ch:1 Dir: PB11 En: none\r\n");
  kprintf ("==============================================\r\n");
  kprintf ("%s\r\n",VERSION);
  kprintf ("\r\n");
#endif

#ifdef ARM_ROBOT
  Stepper_SetupPeripherals(motoTable[0].name, motoTable[0].tim, motoTable[0].tim_ch, &HAL_TIMEx_PWMN_Start, &HAL_TIMEx_PWMN_Stop, M0_DIR_GPIO_Port, M0_DIR_Pin);
  Stepper_SetupPeripherals(motoTable[1].name, motoTable[1].tim, motoTable[1].tim_ch, &HAL_TIM_PWM_Start,    &HAL_TIM_PWM_Stop,    M1_DIR_GPIO_Port, M1_DIR_Pin);
  Stepper_SetupPeripherals(motoTable[2].name, motoTable[2].tim, motoTable[2].tim_ch, &HAL_TIMEx_PWMN_Start, &HAL_TIMEx_PWMN_Stop, M2_DIR_GPIO_Port, M2_DIR_Pin);
#endif

#ifdef BELT_ROBOT
  Stepper_SetupPeripherals(motoTable[3].name, motoTable[3].tim, motoTable[3].tim_ch, &HAL_TIM_PWM_Start,    &HAL_TIM_PWM_Stop,    M3_DIR_GPIO_Port, M3_DIR_Pin);
#endif

  kprintf("Reading settings from internal storage...\r\n");
  Stepper_LoadConfig();

  if (Stepper_GetAccPrescaler(motoTable[MOTO_MAX-1].name) == 0xFFFFFFFF) {
    kprintf("Storage is clean, initializing defaults ...\r\n");
    for (i=0; i < MOTO_MAX; i ++)
    {
      Stepper_InitDefaultState(motoTable[i].name);
    }
    Stepper_SaveConfig();
  }

  for (i=0; i < MOTO_MAX; i++)
  {
    __HAL_TIM_ENABLE_IT(motoTable[i].tim, TIM_IT_UPDATE);
  }

  Serial_InitRxSequence();

  HAL_Delay(100);
  // This will run our StepController timer and enable interrupt for it as well
  HAL_TIM_Base_Start_IT(&htim5);

  // TODO: load settingsfrom FLASH
  for (i=0; i < MOTO_MAX; i++)
  {
    stReq.stepper = motoTable[i].name;
    ExecuteRequest(&stReq);
  }

#ifdef MOTO_EN_USED
  for (i=0; i < MOTO_MAX; i++)
  {
    Stepper_SetEn(motoTable[0].name, 0);
  }
#endif

  Pump_SetEn(1);

#if defined (TEST) 
  Stepper_SetTargetPosition(motoTable[0].name,100);
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
  kprintf("Error_Handler\r\n");
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
