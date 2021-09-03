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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
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
UART_HandleTypeDef huart2;

/* Definitions for UARTRx */
osThreadId_t UARTRxHandle;
const osThreadAttr_t UARTRx_attributes = {
  .name = "UARTRx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskHandler */
osThreadId_t TaskHandlerHandle;
const osThreadAttr_t TaskHandler_attributes = {
  .name = "TaskHandler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ButTask */
osThreadId_t ButTaskHandle;
const osThreadAttr_t ButTask_attributes = {
  .name = "ButTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LedBlinkTask */
osThreadId_t LedBlinkTaskHandle;
const osThreadAttr_t LedBlinkTask_attributes = {
  .name = "LedBlinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for RxQueue */
osMessageQueueId_t RxQueueHandle;
const osMessageQueueAttr_t RxQueue_attributes = {
  .name = "RxQueue"
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};
/* Definitions for b_rateQueue */
osMessageQueueId_t b_rateQueueHandle;
const osMessageQueueAttr_t b_rateQueue_attributes = {
  .name = "b_rateQueue"
};
/* Definitions for UARTTxSemaphore */
osSemaphoreId_t UARTTxSemaphoreHandle;
const osSemaphoreAttr_t UARTTxSemaphore_attributes = {
  .name = "UARTTxSemaphore"
};
/* Definitions for ButEvents */
osEventFlagsId_t ButEventsHandle;
const osEventFlagsAttr_t ButEvents_attributes = {
  .name = "ButEvents"
};
/* USER CODE BEGIN PV */

uint8_t rx_char;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartUARTRx(void *argument);
void StartTTaskHandler(void *argument);
void StartButTask(void *argument);
void StartLedBlinkTask(void *argument);

static void MX_NVIC_Init(void);
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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of UARTTxSemaphore */
  UARTTxSemaphoreHandle = osSemaphoreNew(1, 1, &UARTTxSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of RxQueue */
  RxQueueHandle = osMessageQueueNew (12, sizeof(uint8_t), &RxQueue_attributes);

  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (1, 13, &CommandQueue_attributes);

  /* creation of b_rateQueue */
  b_rateQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &b_rateQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UARTRx */
  UARTRxHandle = osThreadNew(StartUARTRx, NULL, &UARTRx_attributes);

  /* creation of TaskHandler */
  TaskHandlerHandle = osThreadNew(StartTTaskHandler, NULL, &TaskHandler_attributes);

  /* creation of ButTask */
  ButTaskHandle = osThreadNew(StartButTask, NULL, &ButTask_attributes);

  /* creation of LedBlinkTask */
  LedBlinkTaskHandle = osThreadNew(StartLedBlinkTask, NULL, &LedBlinkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of ButEvents */
  ButEventsHandle = osEventFlagsNew(&ButEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    osMessageQueuePut(RxQueueHandle, &rx_char, 1, 0);
	HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		osSemaphoreRelease(UARTTxSemaphoreHandle);

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		osEventFlagsSet(ButEventsHandle, 0x00000004U);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUARTRx */
/**
  * @brief  Function implementing the UARTRx thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUARTRx */
void StartUARTRx(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint8_t temp_rx[2] = { '\0' };
  char rx_msg[13] = { '\0' };
  size_t str_sp = 0;
  osStatus_t status;
  bool send = false;
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(RxQueueHandle, temp_rx, NULL, 2000);

    if (status == osOK)
    {
    	str_sp = sizeof(rx_msg) - strlen(rx_msg) - 1;
    	strncat(rx_msg, (char *)temp_rx, str_sp);

    }

    str_sp = sizeof(rx_msg) - strlen(rx_msg) - 1;

    if (str_sp == 0)
    {
    	rx_msg[strlen(rx_msg)-1] = '\r';
    	rx_msg[strlen(rx_msg)-2] = '\n';
    	send = true;

    }

    else if (status == osErrorTimeout && strlen(rx_msg) > 0 && str_sp >= 2)
    {
    	strncat(rx_msg, "\n\r", str_sp);
    	str_sp = sizeof(rx_msg) - strlen(rx_msg) - 1;
    	send = true;

    }

    else if (status == osErrorTimeout && strlen(rx_msg) > 0 && str_sp == 1)
    {
    	strncat(rx_msg, "\r", str_sp);
    	rx_msg[strlen(rx_msg)-2] = '\n';
    	str_sp = sizeof(rx_msg) - strlen(rx_msg) - 1;
    	send = true;

    }

    else if (rx_msg[strlen(rx_msg)-1] == '\n' || rx_msg[strlen(rx_msg)-1] == '\r')
    {
    	rx_msg[strlen(rx_msg)-1] = '\n';
    	strncat(rx_msg, "\r", str_sp);
    	str_sp = sizeof(rx_msg) - strlen(rx_msg) - 1;
    	send = true;

    }

    if (send && strlen(rx_msg) > 8)
    {
    	osMessageQueuePut(CommandQueueHandle, rx_msg, 1, osWaitForever);
    	osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    	HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Received command:\n\r", 19);

    	str_sp = strlen(rx_msg);
    	osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    	HAL_UART_Transmit_IT(&huart2, (uint8_t *)rx_msg, str_sp);
    	rx_msg[0] = '\0';
    	send = false;

    }

    else if (send)
    {
    	osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    	HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Received command too short:\n\r", 29);
    	str_sp = strlen(rx_msg);
    	osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    	HAL_UART_Transmit_IT(&huart2, (uint8_t *)rx_msg, str_sp);
    	rx_msg[0] = '\0';
    	send = false;
    }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTTaskHandler */
/**
* @brief Function implementing the TaskHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTTaskHandler */
void StartTTaskHandler(void *argument)
{
  /* USER CODE BEGIN StartTTaskHandler */
  osThreadSuspend(LedBlinkTaskHandle);

  char command[13] = { '\0' };
  bool led_b_sus = true;
  int temp_num;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(CommandQueueHandle, command, NULL, osWaitForever);

    if (!strncmp(command, "LED", 3) || !strncmp(command, "led", 3))
    {

    	if (command[4] == 'b' && atoi(command+6) > 0)
    	{
    		if (led_b_sus)
    		{
    			osThreadResume(LedBlinkTaskHandle);
    			temp_num = atoi(command+6);
    			osMessageQueuePut(b_rateQueueHandle, &temp_num, 1, osWaitForever);
    			led_b_sus = !led_b_sus;
    			osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    			HAL_UART_Transmit_IT(&huart2, (uint8_t *)"LED blinking task started\n\r", 27);
    			if (osEventFlagsGet(ButEventsHandle) & 0x00000002U)
    			{
    				osEventFlagsClear(ButEventsHandle, 0x00000002U);
    				osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    				HAL_UART_Transmit_IT(&huart2, (uint8_t *)"LED control with button turned off\n\r", 36);

    			}

    		}

    		else
    		{
    			temp_num = atoi(command+6);
    			osMessageQueuePut(b_rateQueueHandle, &temp_num, 1, osWaitForever);
    			osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    			HAL_UART_Transmit_IT(&huart2, (uint8_t *)"LED blinking rate updated\n\r", 27);

    		}
    	}

    	else if (command[4] == 'b' && command[6] == '0')
    	{
    		if (!led_b_sus)
			{

				osThreadSuspend(LedBlinkTaskHandle);
				led_b_sus = !led_b_sus;
				osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
				HAL_UART_Transmit_IT(&huart2, (uint8_t *)"LED blinking task stopped\n\r", 27);

			}

			else
			{
				osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
				HAL_UART_Transmit_IT(&huart2, (uint8_t *)"LED blinking task already stopped\n\r", 35);

			}

    	}

    	else
    	{
    		osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    		HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Invalid command\n\r", 17);
    	}


    }

    else if (!strncmp(command, "BUT", 3) || !strncmp(command, "but", 3))
    {
    	if (command[4] == 'r' || command[4] == 'R')
    	{
    		if (command[6] == '1')
    		{
    			osEventFlagsSet(ButEventsHandle, 0x00000001U);
    			osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    			HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Button press notifications turned on\n\r", 38);

    		}

    		else
    		{
    			osEventFlagsClear(ButEventsHandle, 0x00000001U);
    			osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    			HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Button press notifications turned off\n\r", 39);

    		}
    	}

    		else if (command[4] == 'l' || command[4] == 'L')
    		{
    			if (command[6] == '1')
				{
					osEventFlagsSet(ButEventsHandle, 0x00000002U);
					osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
					HAL_UART_Transmit_IT(&huart2, (uint8_t *)"LED control with button turned on\n\r", 35);

					if (!led_b_sus)
					{
						osThreadSuspend(LedBlinkTaskHandle);
						led_b_sus = !led_b_sus;
						osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
						HAL_UART_Transmit_IT(&huart2, (uint8_t *)"LED blinking task stopped\n\r", 27);
					}

				}

				else
				{
					osEventFlagsClear(ButEventsHandle, 0x00000002U);
					osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
					HAL_UART_Transmit_IT(&huart2, (uint8_t *)"LED control with button turned off\n\r", 36);
				}
    		}

    }

    else
    {
    	osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
    	HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Invalid command\n\r", 17);

    }

  }
  /* USER CODE END StartTTaskHandler */
}

/* USER CODE BEGIN Header_StartButTask */
/**
* @brief Function implementing the ButTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButTask */
void StartButTask(void *argument)
{
  /* USER CODE BEGIN StartButTask */
  /* Infinite loop */
  for(;;)
  {
	switch (osEventFlagsWait(ButEventsHandle, 0x00000004U, osFlagsWaitAny, osWaitForever))
	{
		case 0x00000005U:
			osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
			HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Button pressed!\n\r", 17);
			break;

		case 0x00000006U:
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			break;

		case 0x00000007U:
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			osSemaphoreAcquire(UARTTxSemaphoreHandle, osWaitForever);
			HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Button pressed!\n\r", 17);

		default:
			break;

	}
  }
  /* USER CODE END StartButTask */
}

/* USER CODE BEGIN Header_StartLedBlinkTask */
/**
* @brief Function implementing the LedBlinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedBlinkTask */
void StartLedBlinkTask(void *argument)
{
  /* USER CODE BEGIN StartLedBlinkTask */
  int rate = 0;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(b_rateQueueHandle, &rate, NULL, 0);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(rate);
  }
  /* USER CODE END StartLedBlinkTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
