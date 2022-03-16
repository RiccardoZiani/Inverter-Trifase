/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI 3.1415926535897932384626433832795
#define _2M_PI 6.283185307179586476925286766559

#define M_PI_2_3 2.0943951023931954923084289221863
#define M_PI_3_2 4.1887902047863909846168578443727

#define SINEDOTS 400
#define SINEDOTS_MAX 1000
#define FREQUENCY_TIM1 80000000
#define START_INVERTER_FREQUENCY 50
#define MIN_INVERTER_FREQUENCY 4
#define MAX_INVERTER_FREQUENCY 400
#define SWITCH_INVERTER_FREQUENCY 30


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;
uint16_t i;
float val = 0;
static uint32_t pp;
static uint32_t mainTick;
static uint32_t mainTick_old;
volatile static uint32_t mainTick_diff;


uint16_t sinetable_X[SINEDOTS_MAX];
uint16_t sinetable_Y[SINEDOTS_MAX];
uint16_t sinetable_Z[SINEDOTS_MAX];
static uint16_t counterslot;
static uint16_t max_counterslot = SINEDOTS;


uint32_t amplitude = 100;
uint32_t frequency = START_INVERTER_FREQUENCY;
uint32_t frequency_old = START_INVERTER_FREQUENCY;
uint32_t auto_reload_counter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);


/* USER CODE BEGIN PFP */

static void update_inveter(void);
static void update_sinetable (uint32_t);

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* Configure LED2 */
  BSP_LED_Init(LED2);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


  update_sinetable(frequency);

/* Start channel 1 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }

  /* Start channel 2 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }

/* Start channel 2 */
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }

/* Start channel 3 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }

/* Start channel 3 */
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }

/* Start tim 6 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    /* start generation Error */
    Error_Handler();
  }











/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */
      mainTick++;
      update_inveter();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    pp++;
    mainTick_diff = mainTick - mainTick_old;
    mainTick_old = mainTick;

    htim1.Instance->CCR1 = sinetable_X[counterslot];
    htim1.Instance->CCR2 = sinetable_Y[counterslot];
    htim1.Instance->CCR3 = sinetable_Z[counterslot];

    counterslot++;
    if (counterslot == max_counterslot)
    {
        counterslot = 0;
    }
}


static void update_inveter(void)
{
    if (frequency_old != frequency)
    {
      if (frequency < MIN_INVERTER_FREQUENCY)
      {
          frequency = MIN_INVERTER_FREQUENCY;

      }
      if (frequency > MAX_INVERTER_FREQUENCY)
      {
          frequency = MAX_INVERTER_FREQUENCY;

      }

      if (frequency < SWITCH_INVERTER_FREQUENCY)
      {
          max_counterslot = SINEDOTS_MAX;
      }
      else
      {
          max_counterslot = SINEDOTS;
      }
      update_sinetable(frequency);
    }

    frequency_old = frequency;
}

static void update_sinetable (uint32_t freq)
{
  uint32_t half_arr;
  auto_reload_counter = (FREQUENCY_TIM1/max_counterslot)/frequency;
  half_arr = auto_reload_counter/2;

  for (i = 0;i < max_counterslot; i++)
  {
      float x;
      x = i;
      //sinetable_X[i] = abs( (int16_t)((sin(_2M_PI*x/400)*3999)));
      //sinetable_Y[i] = abs( (int16_t)((sin(M_PI_2_3 + (_2M_PI*x)/400)*4000)));
      //sinetable_Z[i] = abs( (int16_t)((sin(M_PI_3_2 + (_2M_PI*x)/400)*4000)));

      sinetable_X[i] = half_arr +(int16_t)(sin(_2M_PI*x/max_counterslot)*(half_arr-1));
      sinetable_Y[i] = half_arr +(int16_t)(sin(M_PI_2_3 + (_2M_PI*x)/max_counterslot)*half_arr);
      sinetable_Z[i] = half_arr +(int16_t)(sin(M_PI_3_2 + (_2M_PI*x)/max_counterslot)*half_arr);
  }

  htim1.Instance->ARR = auto_reload_counter-1;
  htim6.Instance->ARR = auto_reload_counter-1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
