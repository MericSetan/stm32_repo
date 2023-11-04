/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include <string.h>
#include "mpu6050.h"
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imu_task_handle */
osThreadId_t imu_task_handleHandle;
const osThreadAttr_t imu_task_handle_attributes = {
  .name = "imu_task_handle",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motor_driver */
osThreadId_t motor_driverHandle;
const osThreadAttr_t motor_driver_attributes = {
  .name = "motor_driver",
  .stack_size = 1023 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void get_encoder_task(void *argument);
void imu_task(void *argument);
void motor_driver_task(void *argument);

/* USER CODE BEGIN PFP */
void print_accel(MPU6050_t *MPU6050);
void print_gyro(MPU6050_t *MPU6050);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(get_encoder_task, NULL, &encoderTask_attributes);

  /* creation of imu_task_handle */
  imu_task_handleHandle = osThreadNew(imu_task, NULL, &imu_task_handle_attributes);

  /* creation of motor_driver */
  motor_driverHandle = osThreadNew(motor_driver_task, NULL, &motor_driver_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1023;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print_accel(MPU6050_t *MPU6050) {
	char buffer[100];
	int32_t len;
	len = sprintf(buffer, "Acc: X:%f Y:%f Z:%f\n", MPU6050->Ax, MPU6050->Ay,
			MPU6050->Az);
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, HAL_MAX_DELAY);
}
void print_gyro(MPU6050_t *MPU6050) {
	char buffer[100];
	int32_t len;
	len = sprintf(buffer, "Gyro: X:%f Y:%f Z:%f\n", MPU6050->Gx, MPU6050->Gy,
			MPU6050->Gz);
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, HAL_MAX_DELAY);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_get_encoder_task */
/**
 * @brief  Function implementing the encoderTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_get_encoder_task */
void get_encoder_task(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	int encoderPosCount = 0;
	char posCount[20];
	int pinALast;
	int aVal;
	bool bCW;
	pinALast = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);
	/* Infinite loop */
	for (;;) {
		aVal = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);
		if (aVal != pinALast) {

			if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) != aVal) {
				encoderPosCount++;
				bCW = 1;
			} else {
				bCW = 0;
				encoderPosCount--;
			}
			//HAL_UART_Transmit(&huart2, (uint8_t*) "Rotated: ", 8,HAL_MAX_DELAY);
			if (bCW) {
				//HAL_UART_Transmit(&huart2, (uint8_t*) "clockwise\n", 9,HAL_MAX_DELAY);
			} else {
				//HAL_UART_Transmit(&huart2, (uint8_t*) "counterclockwise\n", 15,HAL_MAX_DELAY);
			}
			//HAL_UART_Transmit(&huart2, (uint8_t*) "Encoder Position: ", 17,HAL_MAX_DELAY);
			sprintf(posCount, "%d", encoderPosCount);
			HAL_UART_Transmit(&huart2, (uint8_t*) posCount, strlen(posCount),
			HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) "\n", 1, HAL_MAX_DELAY);

		}
		pinALast = aVal;
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_imu_task */
/**
 * @brief Function implementing the imu_task_handle thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_imu_task */
void imu_task(void *argument)
{
  /* USER CODE BEGIN imu_task */
	/* Infinite loop */
	MPU6050_t MPU6050;

	for (;;) {
		MPU6050_Read_Accel(&hi2c1, &MPU6050);
		HAL_Delay(500);
		print_accel(&MPU6050);
	}
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_motor_driver_task */
/**
 * @brief Function implementing the motor_driver thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_motor_driver_task */
void motor_driver_task(void *argument)
{
  /* USER CODE BEGIN motor_driver_task */
	/* Infinite loop */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	// Prescaler = (Saat Hızı) x (İstenen Zaman Dilimi) / (1,000,000)
	// Prescaler = (100,000,000) x (2000) / (1,000,000) = 200
	int speed = 1023;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
	for (;;) {

		HAL_UART_Transmit(&huart2, (uint8_t*) "CCW:\n ", 5,
		HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
		HAL_Delay(5000);

		HAL_UART_Transmit(&huart2, (uint8_t*) "stop\n ", 5,
		HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		HAL_Delay(1000);

		HAL_UART_Transmit(&huart2, (uint8_t*) "CW:\n ", 4,
		HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
		HAL_Delay(5000);

		HAL_UART_Transmit(&huart2, (uint8_t*) "stop\n ", 5,
		HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		HAL_Delay(1000);
	}
  /* USER CODE END motor_driver_task */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
