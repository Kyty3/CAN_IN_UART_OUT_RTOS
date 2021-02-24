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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cmath>
#include <string.h>
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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId UART_CANHandle;
osThreadId CAN_UARTHandle;
/* USER CODE BEGIN PV */
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xSecondBinarySemaphore;

volatile xQueueHandle CAN_UART_Queue;
volatile xQueueHandle UART_CAN_Queue; 

typedef struct{
	uint8_t desired_v[24];
	uint8_t tx_can_data[4];
}from_uart_to_can;

/*CAN variables*/
CAN_TxHeaderTypeDef canTxMes;
CAN_RxHeaderTypeDef canRxMes;
uint32_t TxMailbox = CAN_TX_MAILBOX0;
uint8_t can_in[4];

typedef struct{
	CAN_RxHeaderTypeDef canRxMessage;
	uint8_t can_out[4];
}rxCan;

typedef struct{
		float angle[6];
	}from_stm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void start_send_can(void const * argument);
void start_send_uart(void const * argument);

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	xBinarySemaphore=xSemaphoreCreateBinary();
	xSecondBinarySemaphore=xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	CAN_UART_Queue=xQueueCreate(6, sizeof(rxCan));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UART_CAN */
  osThreadDef(UART_CAN, start_send_can, osPriorityIdle, 0, 128);
  UART_CANHandle = osThreadCreate(osThread(UART_CAN), NULL);

  /* definition and creation of CAN_UART */
  osThreadDef(CAN_UART, start_send_uart, osPriorityIdle, 0, 128);
  CAN_UARTHandle = osThreadCreate(osThread(CAN_UART), NULL);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	sFilterConfig.FilterBank=0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh=0x0000;
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.SlaveStartFilterBank=14;
	
	if(HAL_CAN_ConfigFilter(&hcan,&sFilterConfig)!=HAL_OK){
		Error_Handler();
	}
	
	/*Open CAN*/
	if (HAL_CAN_Start(&hcan)!= HAL_OK)
		{
			Error_Handler();
		}
		

  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/*Interapt CAN_IN functuon*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{		
		
		BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		rxCan canRx;
	
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,  &canRx.canRxMessage, canRx.can_out) != HAL_OK)
		{
			Error_Handler();
		}
		
		xQueueSendToBackFromISR(CAN_UART_Queue, &canRx,&xHigherPriorityTaskWoken);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		xSemaphoreGiveFromISR(xSecondBinarySemaphore, &xHigherPriorityTaskWoken);
		//osDelay(10);
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
	from_uart_to_can message;
	CAN_TxHeaderTypeDef canTxMessage;
	canTxMessage.DLC=4;
	int k=0;//smejenie otchetnogo byte
  /* Infinite loop */
  for(;;)
  {
		if (hdma_usart1_rx.State==HAL_DMA_STATE_READY)
			{
				HAL_UART_Receive_DMA(&huart1,message.desired_v, 24);
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				message.tx_can_data[0]=1;
				message.tx_can_data[1]=1;
				for (int i=0; i<6; i++){
					switch(i)
					{	
						case 0:
							canTxMessage.StdId=0x120;
							k=0;
						break;
						
						case 1:
							canTxMessage.StdId=0x121;
							k=4;
						break;
						
						case 2:
							canTxMessage.StdId=0x122;
							k=8;
						break;
						
						case 3:
							canTxMessage.StdId=0x123;
							k=12;
						break;
										
						case 4:
							canTxMessage.StdId=0x124;
							k=16;
						break;
						
						case 5:
							canTxMessage.StdId=0x125;
							k=20;
						break;
				}
					
					message.tx_can_data[2]=message.desired_v[k+2];
					message.tx_can_data[3]=message.desired_v[k+3];
					
					HAL_CAN_AddTxMessage(&hcan, &canTxMessage,message.tx_can_data, &TxMailbox);
					osDelay(1);
			}
		}
		xSemaphoreGive(xBinarySemaphore);
			
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_send_can */
/**
* @brief Function implementing the UART_CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_send_can */
void start_send_can(void const * argument)
{
  /* USER CODE BEGIN start_send_can */

	
	/* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){ Error_Handler(); } //message in Assigmeng
		if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR) != HAL_OK) {Error_Handler(); }
		//vTaskDelete(start_send_can);
		osDelay(1);
	}
  /* USER CODE END start_send_can */
}

/* USER CODE BEGIN Header_start_send_uart */
/**
* @brief Function implementing the CAN_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_send_uart */
void start_send_uart(void const * argument)
{
  /* USER CODE BEGIN start_send_uart */
	
	rxCan canRx;
	memset(&canRx,0,sizeof(rxCan));
	
	uint8_t str[24]={0};
	from_stm data={0};
	memcpy(str,&data,sizeof(data));

  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(xSecondBinarySemaphore, portMAX_DELAY);
		while(xQueueReceive(CAN_UART_Queue, &canRx,0) !=errQUEUE_EMPTY)
		{
			/*
			if(canRx.canRxMessage.StdId==0x220){
				for(int j=0;j<4;j++){str[j]=canRx.can_out[j];}
			}
			if(canRx.canRxMessage.StdId==0x221){
				for(int j=4;j<8;j++){str[j]=canRx.can_out[j-4];}
			}
			if(canRx.canRxMessage.StdId==0x222){ 
				for(int j=8;j<12;j++){str[j]=canRx.can_out[j-8];}
			}
			if(canRx.canRxMessage.StdId==0x223){
				for(int j=12;j<16;j++){str[j]=canRx.can_out[j-12];}
			}
			if(canRx.canRxMessage.StdId==0x224){
				for(int j=16;j<20;j++){str[j]=canRx.can_out[j-16];}
			}
			if(canRx.canRxMessage.StdId==0x225){
				for(int j=20;j<24;j++){str[j]=canRx.can_out[j-20];}
			}
			*/
			
			if(canRx.canRxMessage.StdId==0x220){
				for(int j=0;j<4;j++){str[j+16]=canRx.can_out[j];}
			}
				
			HAL_UART_Transmit_DMA(&huart1,str,24);
		}
		
  }
  /* USER CODE END start_send_uart */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
