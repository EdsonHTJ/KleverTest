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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACK  0x06
#define NACK 0x17
#define requestSize 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId usart_read_taskHandle;
osThreadId LED_TaskHandle;
osThreadId ADC_TaskHandle;
osMessageQId AdcReadsQueueHandle;
osMessageQId LedQueueHandle;
osMessageQId UsartQueueHandle;
osSemaphoreId BinarySemaphoteUARTHandle;
osSemaphoreId BinarySemaphoreADCHandle;
osSemaphoreId BinarySemaphoreDmaADCHandle;
/* USER CODE BEGIN PV */

uint32_t ADC_read;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void StartUsartTask(void const * argument);
void StartLedTask(void const * argument);
void StartAdcTask(void const * argument);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySemaphoteUART */
  osSemaphoreDef(BinarySemaphoteUART);
  BinarySemaphoteUARTHandle = osSemaphoreCreate(osSemaphore(BinarySemaphoteUART), 1);

  /* definition and creation of BinarySemaphoreADC */
  osSemaphoreDef(BinarySemaphoreADC);
  BinarySemaphoreADCHandle = osSemaphoreCreate(osSemaphore(BinarySemaphoreADC), 1);

  /* definition and creation of BinarySemaphoreDmaADC */
  osSemaphoreDef(BinarySemaphoreDmaADC);
  BinarySemaphoreDmaADCHandle = osSemaphoreCreate(osSemaphore(BinarySemaphoreDmaADC), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of AdcReadsQueue */
  osMessageQDef(AdcReadsQueue, 16, uint16_t);
  AdcReadsQueueHandle = osMessageCreate(osMessageQ(AdcReadsQueue), NULL);

  /* definition and creation of LedQueue */
  osMessageQDef(LedQueue, 16, uint8_t);
  LedQueueHandle = osMessageCreate(osMessageQ(LedQueue), NULL);

  /* definition and creation of UsartQueue */
  osMessageQDef(UsartQueue, 16, uint16_t);
  UsartQueueHandle = osMessageCreate(osMessageQ(UsartQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of usart_read_task */
  osThreadDef(usart_read_task, StartUsartTask, osPriorityIdle, 0, 128);
  usart_read_taskHandle = osThreadCreate(osThread(usart_read_task), NULL);

  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, StartLedTask, osPriorityIdle, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of ADC_Task */
  osThreadDef(ADC_Task, StartAdcTask, osPriorityIdle, 0, 128);
  ADC_TaskHandle = osThreadCreate(osThread(ADC_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance==USART2){
			osSemaphoreRelease(BinarySemaphoteUARTHandle);
	}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	osSemaphoreRelease(BinarySemaphoreDmaADCHandle);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

     osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartUsartTask */
/**
* @brief Function implementing the usart_read_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTask */
void StartUsartTask(void const * argument)
{
  /* USER CODE BEGIN StartUsartTask */
  /* Infinite loop */
  for(;;)
  {
	 uint8_t data[requestSize];
	 osSemaphoreWait(BinarySemaphoteUARTHandle, osWaitForever);
	 HAL_UART_Receive_IT(&huart2, data , requestSize);
	 if(data[0] != 0x01){

			 uint8_t response [5];
			 response[0] = 0x01;
		 	 response[1] = data[1];
			 response[2] = 0x01;
	 		 response[3] = NACK;
	 		 response[4] = response[0] + response[1] + response[2] + response[3] + response[4];
	 		 HAL_UART_Transmit_IT(&huart2, response, 5);


	 }else if(data[3] != data[0]+data[1]+data[2]){
		     uint8_t response [5];
		     response[0] = 0x01;
		 	 response[1] = data[1];
		 	 response[2] = 0x01;
		 	 response[3] = NACK;
		 	 response[4] = response[0] + response[1] + response[2] + response[3] + response[4];
		 	 HAL_UART_Transmit_IT(&huart2, response, 5);


	 }else{
		     uint8_t command;
		     uint8_t response[255];
		     uint8_t response_size;
		     uint16_t Adc_Queue_read;
		     uint8_t AdcMSB;
		     uint8_t AdcLSB;
		 	 switch (data[1]){
		 	 	 case 0x01:
		 	 		 //Turn on LED
		 	 		 command= 0x01;
		 	 		 xQueueSend(LedQueueHandle,&command,osWaitForever);
		 	 		 response[0] = 0x01;
		 	 		 response[1] = data[1];
		 	 		 response[2] = 0x01;
		 	 		 response[3] = ACK;
		 	 	     response[4] = response[0] + response[1] + response[2] + response[3];
		 	 	     response_size= 5;
		 	 		 break;
		 	 	 case 0x02:
		 	 		 //Turn off LED
		 	 		 command= 0x02;
		 	 		 xQueueSend(LedQueueHandle,&command,osWaitForever);
		 	  		 response[0] = 0x01;
		 	         response[1] = data[1];
		 	 		 response[2] = 0x01;
		 	 	     response[3] = ACK;
		 	 		 response[4] = response[0] + response[1] + response[2] + response[3];
		 	 		 response_size= 5;
		 	 	     break;
		 	 	case 0x03:
		 	 		 //Toggle LED
		 	 	     command= 0x03;
		 	 		 xQueueSend(LedQueueHandle,&command,osWaitForever);
		 	 	     response[0] = 0x01;
		 	 	     response[1] = data[1];
		 	 		 response[2] = 0x01;
		 	 		 response[3] = ACK;
		 	 		 response[4] = response[0] + response[1] + response[2] + response[3];
		 	 		 response_size= 5;
		 	 	     break;
		 	 	case 0x04:
		 	 		 for(int i = 0; i < requestSize; i++ ){
		 	 			 	response[i] = data[i];
		 	 		 }
		 	 		 response_size = 4;
		 	 		 break;
		 	 	case 0x05:
		 	 		osSemaphoreRelease(BinarySemaphoreADCHandle);
		 	 		xQueueReceive(AdcReadsQueueHandle, &Adc_Queue_read, osWaitForever);
		 	 		AdcMSB = Adc_Queue_read >> 8;
		 	 		AdcLSB = Adc_Queue_read & 0xFF;

		 	 		response[0] = 0x01;
		 	 		response[1] = data[1];
		 	 		response[2] = 0x02;
		 	 		response[3] = AdcMSB;
		 	 		response[4] = AdcLSB;
		 	 		response[5] = response[0] + response[1] + response[2] + response[3] + response[4];
		 	 		response_size = 6;



		 	 }
		     HAL_UART_Transmit_IT(&huart2, response, response_size);
	 }



	 osDelay(1);

  }
  /* USER CODE END StartUsartTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
	uint8_t command;
	xQueueReceive(LedQueueHandle, &command, osWaitForever);
	switch(command){
	   case 0x01:
		   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
		   break;
	   case 0x02:
		   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		   break;
	   case 0x03:
	   	   HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	   	   break;

	}
    osDelay(1);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartAdcTask */
/**
* @brief Function implementing the ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdcTask */
void StartAdcTask(void const * argument)
{
  /* USER CODE BEGIN StartAdcTask */
  /* Infinite loop */



  for(;;)
  {
    osSemaphoreWait(BinarySemaphoreADCHandle, osWaitForever);



	HAL_ADC_Start_DMA(&hadc1, &ADC_read, 1);
	osSemaphoreWait(BinarySemaphoreDmaADCHandle, osWaitForever);
	uint32_t ADCRT = ADC_read;
	xQueueSend(AdcReadsQueueHandle,&ADC_read,osWaitForever);

    osDelay(10);
  }
  /* USER CODE END StartAdcTask */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
