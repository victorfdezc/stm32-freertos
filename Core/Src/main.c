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
// #include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
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
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
// osThreadId_t defaultTaskHandle;
// const osThreadAttr_t defaultTask_attributes = {
//   .name = "defaultTask",
//   .stack_size = 128 * 4,
//   // .priority = (osPriority_t) osPriorityNormal,
// };
/* USER CODE BEGIN PV */
static SemaphoreHandle_t mutex;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vTask1( void *pvParameters )
{
  const char *pcTaskName = "Task 1";
  char buf[50];
  uint32_t ul=0; /* volatile to ensure ul is not optimized away. */
  /* As per most tasks, this task is implemented in an infinite loop. */
  for( ;; )
  {
    sprintf(buf,"%s: %d \r\n",pcTaskName,ul);
    // Take the mutex
    xSemaphoreTake(mutex, portMAX_DELAY);
    /* Print out the name of this task. */
    HAL_UART_Transmit(&huart1, buf, sizeof(char)*strlen(buf), HAL_MAX_DELAY);
    // HAL_UART_Transmit_DMA(&huart1, pcTaskName, sizeof(pcTaskName)); //Non-blocking mode with DMA
    // Release the mutex so that the creating function can finish
    xSemaphoreGive(mutex);
    vTaskDelay(1000);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    ul++;
    /* Delay for a period. */
    // for( ul = 0; ul < 1000; ul++ )
    // {
    //   /* This loop is just a very crude delay implementation. There is
    //   nothing to do in here. Later examples will replace this crude
    //   loop with a proper delay/sleep function. */
    // }
  }
}

void vTask2( void *pvParameters )
{
  const char *pcTaskName = "Task 2";
  char buf[50];
  uint32_t ul=0; /* volatile to ensure ul is not optimized away. */
  /* As per most tasks, this task is implemented in an infinite loop. */
  for( ;; )
  {
    sprintf(buf,"%s: %d \r\n",pcTaskName,ul);
    // Take the mutex
    xSemaphoreTake(mutex, portMAX_DELAY);
    /* Print out the name of this task. */
    HAL_UART_Transmit(&huart1, buf, sizeof(char)*strlen(buf), HAL_MAX_DELAY);
    // Release the mutex so that the creating function can finish
    xSemaphoreGive(mutex);
    // HAL_UART_Transmit_DMA(&huart1, pcTaskName, sizeof(pcTaskName)); //Non-blocking mode with DMA
    vTaskDelay(2000);
    ul++;
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    /* Delay for a period. */
    // for( ul = 0; ul < 500; ul++ )
    // {
    //   /* This loop is just a very crude delay implementation. There is
    //   nothing to do in here. Later examples will replace this crude
    //   loop with a proper delay/sleep function. */
    // }
  }
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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  // osKernelInitialize();

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
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  // osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // HAL_Delay(1000);
  // // vTaskDelay(1000);
  // // for(int i=0;i<100000000;i++);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // HAL_Delay(1000);
  // // vTaskDelay(1000);
  // // for(int i=0;i<100000000;i++);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // HAL_Delay(1000);
  // // vTaskDelay(1000);
  // // for(int i=0;i<100000000;i++);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // HAL_Delay(1000);
  // // vTaskDelay(1000);
  // // for(int i=0;i<100000000;i++);
  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // HAL_Delay(1000);
  // vTaskDelay(1000);
  // for(int i=0;i<100000000;i++);

  // Create mutex before starting tasks
  mutex = xSemaphoreCreateMutex();
  // Give the mutex
  xSemaphoreGive(mutex);

  char data[50] = {"Creating tasks...\r\n"};
  // uint8_t datalen = strlen(data)+48; //suma "0" en ASCII para que se imprima correctamente
  HAL_UART_Transmit(&huart1, &data, sizeof(char)*strlen(data), HAL_MAX_DELAY);

  xTaskCreate( vTask1, /* Pointer to the function that implements the task. */
              "Task 1",/* Text name for the task. This is to facilitate
              debugging only. */
              1000, /* Stack depth - small microcontrollers will use much
              less stack than this. */
              NULL, /* This example does not use the task parameter. */
              1, /* This task will run at priority 1. */
              NULL ); /* This example does not use the task handle. */
  /* Create the other task in exactly the same way and at the same priority. */
  xTaskCreate( vTask2, "Task 2", 1000, NULL, 1, NULL );
  /* Start the scheduler so the tasks start executing. */

  strcpy(data, "Starting Scheduler... \r\n");
  HAL_UART_Transmit(&huart1, &data, sizeof(char)*strlen(data), HAL_MAX_DELAY);
  vTaskStartScheduler();
  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
