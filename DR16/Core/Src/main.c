/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "DR16_control.h"
#include "stdio.h"
#include "freertos.h"
#include "task.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define  LENGTH  18

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];
extern uint16_t count_1;
extern uint16_t count_2;

uint8_t RxBuffer[LENGTH];
uint8_t RecCount = 0;
uint8_t RxFlag = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float feedbackValue;
float feedbackValue1;
float targetValue = 0;

uint32_t RxID;
uint8_t RxLength = 8;
uint8_t RxData[8];

int16_t Speed;

float alpha = 0.05;  // 平滑因子

QueueHandle_t QueueHandler;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId controlHandle;
osSemaphoreId myBinarySem01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void DR16_control(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
  float kp, ki, kd;            // 三个系数
  float error, lastError;      // 误差、上次误�??????
  float integral, maxIntegral; // 积分、积分限�??????
  float output, maxOutput;     // 输出、输出限�??????
} PID;

// 用于初始化pid参数的函�??????
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->maxIntegral = maxI;
  pid->maxOutput = maxOut;
}

// 进行�??????次pid计算
// 参数�??????(pid结构�??????,目标�??????,反馈�??????)，计算结果放在pid结构体的output成员成员�??????
void PID_Calc(PID *pid, float reference, float feedback)
{
  // 更新数据
  pid->lastError = pid->error;       // 将旧error存起�??????
  pid->error = reference - feedback; // 计算新error
  // 计算微分
  static float dout;
  dout = (pid->error - pid->lastError) * pid->kd;
  // 计算比例
  static float pout;
  pout = pid->error * pid->kp;
  // 计算积分
  pid->integral += pid->error;
  static float iout;
  iout = pid->integral * pid->ki;
  // 积分限幅
  if (pid->integral > pid->maxIntegral)
    pid->integral = pid->maxIntegral;
  else if (pid->integral < -pid->maxIntegral)
    pid->integral = -pid->maxIntegral;
  // 计算输出
  pid->output = pout + dout + iout;
  // 输出限幅
  if (pid->output > pid->maxOutput)
    pid->output = pid->maxOutput;
  else if (pid->output < -pid->maxOutput)
    pid->output = -pid->maxOutput;
}

float emaFilter(float input, float *prev_ema, float alpha)
{
  // 计算新的 EMA �??
  *prev_ema = alpha * input + (1.0f - alpha) * (*prev_ema);
  return *prev_ema;
}

void FilterInit(void)
{
  CAN_FilterTypeDef CAN_FilterInitStructure;
  CAN_FilterInitStructure.FilterActivation = ENABLE;
  CAN_FilterInitStructure.FilterBank = 0;
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  CAN_FilterInitStructure.FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.FilterIdLow = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterInitStructure.SlaveStartFilterBank = 0;
  HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure);

  if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure) != HAL_OK)
  {
    //  printf("CAN1 ConfigFilter Fail!!\r\n");
  }
  else
  {
    // printf("CAN1 ConfigFilter SUCCESS!!\r\n");
  }
}

void CAN1_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
  CAN_TxHeaderTypeDef TxMessage = {0};
  uint8_t Tx_Buffer[8] = {0};
  uint32_t box = 0;
  TxMessage.StdId = ID;
  TxMessage.ExtId = ID;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = Length;
  TxMessage.TransmitGlobalTime = DISABLE;
  for (uint8_t i = 0; i < Length; i++)
  {
    Tx_Buffer[i] = Data[i];
  }
  HAL_StatusTypeDef TransmitMailbox;
  TransmitMailbox = HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Tx_Buffer, &box);
  if (TransmitMailbox != HAL_OK)
  {
    //	printf("Transmit Error!");
  }
  else
  {
    // printf("Transmit Success!\r\n");
  }

  // vTaskDelayUntil(&xLastWakeTime,1000);
}

uint8_t CAN1_ReceiveFlag(void)
{
  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0)
  {
    //	printf("CAN1 has some Message");
    return 1;
  }
  return 0;
}

void CAN1_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
  //		FilterInit();

  CAN_RxHeaderTypeDef rceStu = {0};
  if (CAN1_ReceiveFlag() != 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rceStu, Data) == HAL_OK)
    {
      // printf("rceStu.DLC: %d\r\n",rceStu.DLC);
      // printf("rceStu.ExtId: %d\r\n",rceStu.ExtId);
      // printf("rceStu.StdId: %x\r\n",rceStu.StdId);
      // printf("rceStu.Timestamp: %d\r\n",rceStu.Timestamp);
      for (uint8_t i = 0; i < rceStu.DLC; i++)
      {
        // printf(" %x",Data[i]);
      }
      //printf("\r\n");
    }
  }
  else
  {
    // printf("No CAN1 INFO!\r\n");
  }
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
 // if(hcan->Instance == CAN1)
 // {
//   CAN1_Receive(&RxID, &RxLength, RxData);
//  }
 
//}

 PID mypid = {0};
 
 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  QueueHandler = xQueueCreate(20, 20);
  if (QueueHandler == NULL)
  {
    printf("error");
  }
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of control */
  osThreadDef(control, DR16_control, osPriorityNormal, 0, 128);
  controlHandle = osThreadCreate(osThread(control), NULL);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART2)
	{
		HAL_UART_Receive_DMA(&huart2,(uint8_t*)RxBuffer,LENGTH);
	}
}

void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	xSemaphoreGiveFromISR(myBinarySem01Handle,&xHigherPriorityTaskWoken);
//	xSemaphoreGive(myBinarySem01Handle);
	
	RxFlag = 1;
	HAL_UART_DMAStop(&huart2);			//关闭DMA，每次空闲都会进函数
	
	  if(RxFlag == 1)
	  {
			RxFlag = 0;
		  RecCount = LENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		//	printf("%s\r\n",RxBuffer);	
		  RecCount = 0;
			HAL_UART_Receive_DMA(&huart2,(uint8_t*)RxBuffer,LENGTH);		
	  }
			
}

int fputc(int ch, FILE *f)               //重定向fputc函数
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}

int fgetc(FILE *f)               //重定向fgetc函数
{
	uint8_t ch;
	HAL_UART_Receive(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)		//遥控保护
{
  /* USER CODE BEGIN 5 */
	uint32_t tick_1 = 0;
	uint32_t tick_2 = 0;
	uint32_t tick_interval = 0;
  /* Infinite loop */
  for(;;)
  { 
		tick_2 = HAL_GetTick();		//获取当前时刻的tick值
		if(xSemaphoreTake(myBinarySem01Handle,100) == pdTRUE)
		tick_1 = HAL_GetTick();			//再次获取
		tick_interval = tick_1 - tick_2;	//判断两次tick之间的时间差

			if(tick_interval <= 100)		//如果小于等于100，说明在等待时间内获取了信号量
			{
				RemoteDataProcess((uint8_t*)RxBuffer);
			}
			else						//如果大于100，说明为超时退出
			{
				memset(&RC_CtrlData, 0, sizeof(RC_CtrlData));
			}
			xQueueSend(QueueHandler , &RC_CtrlData, 0);
	
    vTaskDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DR16_control */
/**
* @brief Function implementing the control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DR16_control */
void DR16_control(void const * argument)
{
  /* USER CODE BEGIN DR16_control */
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);			//使能中断
  HAL_UART_Receive_DMA(&huart2,(uint8_t*)RxBuffer,LENGTH);		//�?启DMA中断
	
	uint32_t TxID = 0x1FF;
  uint8_t TxLength = 8;
  uint8_t TxData[8] = {0};
	uint8_t TxData_1[8] = {0};

  HAL_CAN_Start(&hcan1);

  FilterInit();

  PID_Init(&mypid, 3, 1, 5, 20000, 15000);

  float prev_ema = 0; // 初始 EMA �?

  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(QueueHandler, &RC_CtrlData, 0);
		
		TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
		
		targetValue = RC_CtrlData.rc.ch0 * 25000 / 1300;

    CAN1_Receive(&RxID, &RxLength, RxData);
    Speed = (RxData[2] << 8) | RxData[3];

    feedbackValue = Speed; // 这里获取到被控对象的反馈�?

    float ema_result = emaFilter(feedbackValue, &prev_ema, alpha);

    PID_Calc(&mypid, targetValue, ema_result); // 进行PID计算，结果在output成员变量

    TxData[0] = (((int16_t)mypid.output) >> 8) & 0xff; // 右移八位是因�?16位数据只有后面八位可以存�?8位的数组
    TxData[1] = ((int16_t)mypid.output) & 0xff;

		CAN1_Transmit(TxID, TxLength, TxData);
		
     vTaskDelayUntil(&xLastWakeTime, 1);
  }
  /* USER CODE END DR16_control */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
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
