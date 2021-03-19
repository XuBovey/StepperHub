#ifndef __COMMON_H__
#define __COMMON_H__

#include "stm32f4xx_hal.h"
#include "serial.h"

// #define VERSION "V0.1.3.02 2021-2-19"
#define VERSION "V0.1.3.04 2021-3-18"

// #define MOTO_EN_USED

#define ARM_ROBOT
#define BELT_ROBOT

#ifdef ARM_ROBOT
#define ARM_MOTO 3
#else
#define ARM_MOTO 0
#endif

#ifdef BELT_ROBOT
#define BELT_MOTO 1
#else
#define BELT_MOTO 0
#endif

#define MOTO_MAX (ARM_MOTO + BELT_MOTO)

typedef struct
{
  char name;
  TIM_HandleTypeDef * tim;
  int tim_ch;
}_moto;

typedef struct
{
    uint8_t last_state;
    uint8_t current_state;
    uint8_t active_state;
    GPIO_TypeDef* port;
    uint16_t pin;
}_moto_limited;

#endif //__COMMON_H__