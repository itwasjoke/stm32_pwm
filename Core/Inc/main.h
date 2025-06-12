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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    TIM_HandleTypeDef* htim;
    TIM_TypeDef* tim_instance;
    uint32_t ic_channel_rising;
    uint32_t ic_channel_falling;
    uint32_t ic_channel_rising_active;
	uint32_t ic_channel_falling_active;

    volatile uint32_t overflow_counter;
    volatile uint64_t last_rising_edge_ticks;
    volatile uint64_t period_total_ticks;
    volatile uint64_t duty_cycle_total_ticks;
    volatile uint8_t capture_state;
} PWM_Meas_Channel_t;
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
float GetPeriodSeconds(uint8_t channel_idx);
float GetFrequencyHz(uint8_t channel_idx);
float GetDutyCyclePercent(uint8_t channel_idx);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define TIMER_MAX_VALUE 0xFFFFUL
#define TIMER_CLOCK_FREQ_HZ 84000000.0f
#define TICKS_TO_SECONDS (1000000.0f / TIMER_CLOCK_FREQ_HZ)
#define NUM_PWM_CHANNELS 8
#define OVERFLOW_THRESHOLD_1_SEC (uint32_t)(TIMER_CLOCK_FREQ_HZ / (TIMER_MAX_VALUE + 1) + 1)
extern PWM_Meas_Channel_t pwm_channels[NUM_PWM_CHANNELS];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
