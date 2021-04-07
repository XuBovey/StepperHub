/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "stepperController.h"
#include "serial.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
typedef struct {
    unsigned int crash_time;
    unsigned int is_crash;
    /* register info*/
    unsigned long stacked_r0;
    unsigned long stacked_r1;  
    unsigned long stacked_r2; 
    unsigned long stacked_r3;    
    unsigned long stacked_r12;  
    unsigned long stacked_lr;  
    unsigned long stacked_pc;  
    unsigned long stacked_psr;  
    unsigned long SHCSR;  
    unsigned long MFSR;  
    unsigned long BFSR;   
    unsigned long UFSR;  
    unsigned long HFSR;  
    unsigned long DFSR;  
    unsigned long MMAR;  
    unsigned long BFAR;
} System_Crash_Info;

static System_Crash_Info crash_info;
void hard_fault_handler_c(unsigned int * hardfault_args)
{
  memset(&crash_info, 0, sizeof(System_Crash_Info));

  crash_info.is_crash = 1;
  crash_info.crash_time = (unsigned int)HAL_GetTick();

  crash_info.stacked_r0 = ((unsigned long) hardfault_args[0]);
  crash_info.stacked_r1 = ((unsigned long) hardfault_args[1]);
  crash_info.stacked_r2 = ((unsigned long) hardfault_args[2]);
  crash_info.stacked_r3 = ((unsigned long) hardfault_args[3]);
  crash_info.stacked_r12 = ((unsigned long) hardfault_args[4]);
  crash_info.stacked_lr = ((unsigned long) hardfault_args[5]);
  crash_info.stacked_pc = ((unsigned long) hardfault_args[6]);
  crash_info.stacked_psr = ((unsigned long) hardfault_args[7]);

  crash_info.MFSR = (*((volatile unsigned char *)(0xE000ED28))); //存储器管理fault状态寄存器
  crash_info.BFSR = (*((volatile unsigned char *)(0xE000ED29))); //总线fault状态寄存器
  crash_info.UFSR = (*((volatile unsigned short int *)(0xE000ED2A)));//用法fault状态寄存器
  crash_info.HFSR = (*((volatile unsigned long *)(0xE000ED2C)));  //硬fault状态寄存器
  crash_info.DFSR = (*((volatile unsigned long *)(0xE000ED30))); //调试fault状态寄存器
  crash_info.MMAR = (*((volatile unsigned long *)(0xE000ED34))); //存储管理地址寄存器
  crash_info.BFAR = (*((volatile unsigned long *)(0xE000ED38))); //总线fault地址寄存器

  kprintf("Error: HardFault_Handler hard_fault_handler_c\nError: R0 = 0x%08x\nError: R1 = 0x%08x\nError: R2 = 0x%08x\nError: R3 = 0x%08x\nError: R12 = 0x%08x\nError: lr = 0x%08x\nError: pc = 0x%08x\nError: psr = 0x%08x\nError: MFSR = 0x%08x\nError: BFSR = 0x%08x\nError: UFSR = 0x%08x\nError: HFSR = 0x%08x\nError: DFSR = 0x%08x\nError: MMAR = 0x%08x\nError: BFAR = 0x%08x\n"
          ,crash_info.stacked_r0
          ,crash_info.stacked_r1
          ,crash_info.stacked_r2
          ,crash_info.stacked_r3
          ,crash_info.stacked_r12
          ,crash_info.stacked_lr
          ,crash_info.stacked_pc
          ,crash_info.stacked_psr
          ,crash_info.MFSR
          ,crash_info.BFSR
          ,crash_info.UFSR
          ,crash_info.HFSR
          ,crash_info.DFSR
          ,crash_info.MMAR
          ,crash_info.BFAR);

  // stopAllStepper();
  // Stepper_SetEn('X', 0);
  // Stepper_SetEn('Y', 0);
  // Stepper_SetEn('Z', 0);
  while(1);
}

/* hard fault interrupt handler */
// void _int_hardfault_isr( )
// {
// __asm("TST LR, #4");
// __asm("ITE EQ");
// __asm("MRSEQ R0,MSP");
// __asm("MRSNE R0,PSP");
// __asm("B hard_fault_handler_c");
// }

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim13;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern _moto motoTable[];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
//  kprintf("Error: HardFault_Handler\n");
__asm("TST LR, #4");
__asm("ITE EQ");
__asm("MRSEQ R0,MSP");
__asm("MRSNE R0,PSP");
__asm("B hard_fault_handler_c");
  // _int_hardfault_isr();
  // stopAllStepper();
  // Stepper_SetEn('X', 0);
  // Stepper_SetEn('Y', 0);
  // Stepper_SetEn('Z', 0);
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  kprintf("Error: MemManage_Handler\n");
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  kprintf("Error: BusFault_Handler\n");
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  kprintf("Error: UsageFault_Handler\n");
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */
  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
  static int count = 0;
  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))
  {
    if (__HAL_TIM_GET_ITSTATUS(&htim1, TIM_IT_UPDATE))
    {
      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
      Stepper_PulseTimerUpdate(motoTable[2].name);
      if(count ++ > 200)
      {
        count = 0;
        led_toggle(LED_BLU_GPIO_Port, LED_BLU_Pin);
      }
    }
  }
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  static int count = 0;
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE))
  {
    if (__HAL_TIM_GET_ITSTATUS(&htim2, TIM_IT_UPDATE))
    {
      __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
      Stepper_PulseTimerUpdate(motoTable[1].name);
      if(count ++ > 200)
      {
        count = 0;
        led_toggle(LED_YEL_GPIO_Port, LED_GRN_Pin);
      }
    }
  }
  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
  static int count = 0;
  if (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE))
  {
    if (__HAL_TIM_GET_ITSTATUS(&htim8, TIM_IT_UPDATE))
    {
      __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
      Stepper_PulseTimerUpdate(motoTable[0].name);

      if(count ++ > 200)
      {
        led_toggle(LED_RED_GPIO_Port, LED_RED_Pin);
        count = 0;
      }
    }
  }

  if (__HAL_TIM_GET_FLAG(&htim13, TIM_FLAG_UPDATE))
  {
    if (__HAL_TIM_GET_ITSTATUS(&htim13, TIM_IT_UPDATE))
    {
      __HAL_TIM_CLEAR_FLAG(&htim13, TIM_FLAG_UPDATE);
      Stepper_PulseTimerUpdate(motoTable[3].name);
    }
  }
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
