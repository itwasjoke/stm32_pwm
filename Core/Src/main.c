/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
I2C_LCD_HandleTypeDef lcd1;
volatile uint8_t cur_channel = 0;
PWM_Meas_Channel_t pwm_channels[NUM_PWM_CHANNELS] = {
		{
				.htim = &htim1,
				.tim_instance = TIM1,
				.ic_channel_rising = TIM_CHANNEL_1,
				.ic_channel_falling = TIM_CHANNEL_2,
				.ic_channel_rising_active = HAL_TIM_ACTIVE_CHANNEL_1,
				.ic_channel_falling_active = HAL_TIM_ACTIVE_CHANNEL_2,
				.overflow_counter = 0,
				.last_rising_edge_ticks = 0,
				.period_total_ticks = 0,
				.duty_cycle_total_ticks = 0,
				.capture_state = 0
		},
		{
				.htim = &htim1,
				.tim_instance = TIM1,
				.ic_channel_rising = TIM_CHANNEL_3,
				.ic_channel_falling = TIM_CHANNEL_4,
				.ic_channel_rising_active = HAL_TIM_ACTIVE_CHANNEL_3,
				.ic_channel_falling_active = HAL_TIM_ACTIVE_CHANNEL_4,
				.overflow_counter = 0,
				.last_rising_edge_ticks = 0,
				.period_total_ticks = 0,
				.duty_cycle_total_ticks = 0,
				.capture_state = 0
		},
		{
				.htim = &htim2,
				.tim_instance = TIM2,
				.ic_channel_rising = TIM_CHANNEL_3,
				.ic_channel_falling = TIM_CHANNEL_4,
				.ic_channel_rising_active = HAL_TIM_ACTIVE_CHANNEL_3,
				.ic_channel_falling_active = HAL_TIM_ACTIVE_CHANNEL_4,
				.overflow_counter = 0,
				.last_rising_edge_ticks = 0,
				.period_total_ticks = 0,
				.duty_cycle_total_ticks = 0,
				.capture_state = 0
		},
		{
				.htim = &htim3,
				.tim_instance = TIM3,
				.ic_channel_rising = TIM_CHANNEL_1,
				.ic_channel_falling = TIM_CHANNEL_2,
				.ic_channel_rising_active = HAL_TIM_ACTIVE_CHANNEL_1,
				.ic_channel_falling_active = HAL_TIM_ACTIVE_CHANNEL_2,
				.overflow_counter = 0,
				.last_rising_edge_ticks = 0,
				.period_total_ticks = 0,
				.duty_cycle_total_ticks = 0,
				.capture_state = 0
		},
		{
				.htim = &htim3,
				.tim_instance = TIM3,
				.ic_channel_rising = TIM_CHANNEL_3,
				.ic_channel_falling = TIM_CHANNEL_4,
				.ic_channel_rising_active = HAL_TIM_ACTIVE_CHANNEL_3,
				.ic_channel_falling_active = HAL_TIM_ACTIVE_CHANNEL_4,
				.overflow_counter = 0,
				.last_rising_edge_ticks = 0,
				.period_total_ticks = 0,
				.duty_cycle_total_ticks = 0,
				.capture_state = 0
		},
		{
				.htim = &htim4,
				.tim_instance = TIM4,
				.ic_channel_rising = TIM_CHANNEL_1,
				.ic_channel_falling = TIM_CHANNEL_2,
				.ic_channel_rising_active = HAL_TIM_ACTIVE_CHANNEL_1,
				.ic_channel_falling_active = HAL_TIM_ACTIVE_CHANNEL_2,
				.overflow_counter = 0,
				.last_rising_edge_ticks = 0,
				.period_total_ticks = 0,
				.duty_cycle_total_ticks = 0,
				.capture_state = 0
		},
		{
				.htim = &htim4,
				.tim_instance = TIM4,
				.ic_channel_rising = TIM_CHANNEL_3,
				.ic_channel_falling = TIM_CHANNEL_4,
				.ic_channel_rising_active = HAL_TIM_ACTIVE_CHANNEL_3,
				.ic_channel_falling_active = HAL_TIM_ACTIVE_CHANNEL_4,
				.overflow_counter = 0,
				.last_rising_edge_ticks = 0,
				.period_total_ticks = 0,
				.duty_cycle_total_ticks = 0,
				.capture_state = 0
		},
		{
				.htim = &htim9,
				.tim_instance = TIM9,
				.ic_channel_rising = TIM_CHANNEL_1,
				.ic_channel_falling = TIM_CHANNEL_2,
				.ic_channel_rising_active = HAL_TIM_ACTIVE_CHANNEL_1,
				.ic_channel_falling_active = HAL_TIM_ACTIVE_CHANNEL_2,
				.overflow_counter = 0,
				.last_rising_edge_ticks = 0,
				.period_total_ticks = 0,
				.duty_cycle_total_ticks = 0,
				.capture_state = 0
		},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float GetPeriodSeconds(uint8_t channel_idx) {
    if (channel_idx >= NUM_PWM_CHANNELS) {
        return 0.0f;
    }
    return (float)pwm_channels[channel_idx].period_total_ticks * TICKS_TO_SECONDS;
}

float GetFrequencyHz(uint8_t channel_idx) {
    if (channel_idx >= NUM_PWM_CHANNELS || pwm_channels[channel_idx].period_total_ticks == 0) {
        return 0.0f;
    }
    return TIMER_CLOCK_FREQ_HZ / (float)pwm_channels[channel_idx].period_total_ticks;
}

float GetDutyCyclePercent(uint8_t channel_idx) {
    if (channel_idx >= NUM_PWM_CHANNELS || pwm_channels[channel_idx].period_total_ticks == 0) {
        return 0.0f;
    }
    return ((float)pwm_channels[channel_idx].duty_cycle_total_ticks / pwm_channels[channel_idx].period_total_ticks) * 100.0f;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  lcd1.hi2c = &hi2c1;
  lcd1.address = 0x4E;
  lcd_init(&lcd1);
  lcd_puts(&lcd1, "loading...");

  // htim1
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
	  Error_Handler();
  }

  // htim2
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
	  Error_Handler();
  }

  // htim3
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
	  Error_Handler();
  }

  // htim4
  if (HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
	  Error_Handler();
  }

  // htim9
  if (HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim9) != HAL_OK)
  {
	  Error_Handler();
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_clear(&lcd1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  float period = GetPeriodSeconds(cur_channel);
	  char period_str[15];
	  snprintf(period_str, sizeof(period_str), "%.2f", period);

	  float duty = GetDutyCyclePercent(cur_channel);
	  char duty_str[15];
	  snprintf(duty_str, sizeof(duty_str), "%.2f", duty);

	  char channel_number_str[15];
	  sprintf(channel_number_str, "%u", cur_channel+1);

	  lcd_gotoxy(&lcd1, 0, 0);
	  lcd_puts(&lcd1, period_str);
	  lcd_gotoxy(&lcd1, 0, 1);
	  lcd_puts(&lcd1, duty_str);
	  lcd_gotoxy(&lcd1, 15, 1);
	  lcd_puts(&lcd1, channel_number_str);
	  if (cur_channel == 7)
	  {
		  cur_channel = 0;
	  }
	  else
	  {
		  cur_channel++;
	  }
	  for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++)
	  {
		  if (pwm_channels[i].overflow_counter >= OVERFLOW_THRESHOLD_1_SEC)
		  {
			  pwm_channels[i].overflow_counter = 0;
			  pwm_channels[i].last_rising_edge_ticks = 0;
			  pwm_channels[i].period_total_ticks = 0;
			  pwm_channels[i].duty_cycle_total_ticks = 0;
			  pwm_channels[i].capture_state = 0;
		  }
	  }
	  HAL_Delay(2000);
	  lcd_clear(&lcd1);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 49;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
//
//	  GPIO_InitStruct.Pin = GPIO_PIN_6;
//	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
	  lcd_puts(&lcd1, "error");
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
