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
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float outerFeedback;
float outerFeedback_1;
float outerTarget_first = 0;
float outerTarget = 0;
float innerFeedback;

uint32_t RxID;
uint8_t RxLength = 8;
uint8_t RxData[8];

int16_t Speed;
uint16_t Angel_first;
int8_t Angel;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId PIDHandle;
osThreadId myTask03Handle;
/* USER CODE BEGIN PV */
QueueHandle_t QueueHandler;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const *argument);
void StartTask02(void const *argument);
void StartTask03(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
  float kp, ki, kd;            // 三个系数
  float error, lastError;      // 误差、上次误�?????
  float integral, maxIntegral; // 积分、积分限�?????
  float output, maxOutput;     // 输出、输出限�?????
} PID;

// 用于初始化pid参数的函�?????
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->maxIntegral = maxI;
  pid->maxOutput = maxOut;
}

// 进行�?????次pid计算
// 参数�?????(pid结构�?????,目标�?????,反馈�?????)，计算结果放在pid结构体的output成员成员�?????
void PID_Calc(PID *pid, float reference, float feedback)
{
  // 更新数据
  pid->lastError = pid->error;       // 将旧error存起�?????
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

//串级PID的结构体，包含两个单级PID
typedef struct
{
    PID inner; //内环
    PID outer; //外环
    float output; //串级输出，等于inner.output
}CascadePID;
 
//串级PID的计算函数
//参数(PID结构体,外环目标值,外环反馈值,内环反馈值)
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb)
{
    PID_Calc(&pid->outer, outerRef, outerFdb); //计算外环
    PID_Calc(&pid->inner, pid->outer.output, innerFdb); //计算内环
    pid->output = pid->inner.output; //内环输出就是串级PID的输出
}
 
CascadePID mypid = {0}; //创建串级PID结构体变量

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

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
 
int fgetc(FILE *f)
{
  uint8_t ch;
  HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)


{
  /* USER CODE BEGIN 1 */
  QueueHandler = xQueueCreate(8, 8);
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of PID */
  osThreadDef(PID, StartTask02, osPriorityNormal, 0, 128);
  PIDHandle = osThreadCreate(osThread(PID), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN 5 */
  vTaskDelete(NULL);

  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the PID thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const *argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint32_t TxID = 0x1FF;
  uint8_t TxLength = 8;
  uint8_t TxData[8] = {0};            //定义参数，目的是发送数据给电机

//   uint32_t RxID;
//   uint8_t RxLength = 8;
//   uint8_t RxData[8];

  HAL_CAN_Start(&hcan1);

  FilterInit();

  PID_Init(&mypid.inner, 0, 0, 0, 1000, 1000); //初始化内环参数
  PID_Init(&mypid.outer, 0, 0, 0, 1000, 1000); //初始化外环参数

  uint8_t TeBuffer[8];         //定义数组传入队列

  /* Infinite loop */
  for (;;)
  {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN1_Receive(&RxID, &RxLength, RxData);

    Angel_first = (RxData[0] << 8) | RxData[1];
    Speed = (RxData[2] << 8) | RxData[3];

    Angel = Angel_first * 360 / 8191 ;

    outerFeedback = Angel; // 这里获取到被控对象的反馈值
    innerFeedback = Speed;//  获取内环反馈值

    int8_t *p = &Angel;

    TeBuffer[0] = *p;

    PID_CascadeCalc(&mypid, outerTarget, outerFeedback, innerFeedback); //进行PID计算

    TxData[0] = (((int16_t)mypid.output) >> 8) & 0xff; // 右移八位是因为16位数据只有后面八位可以存入8位的数组
    TxData[1] = ((int16_t)mypid.output) & 0xff;

    CAN1_Transmit(TxID, TxLength, TxData);

    xQueueSend(QueueHandler, TeBuffer, 0);

    // printf("Output:%f\n",mypid.output);
    // printf("%f,%f\n",targetValue,feedbackValue);

    vTaskDelayUntil(&xLastWakeTime, 1); // 延时
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const *argument)
{
  /* USER CODE BEGIN StartTask03 */

  /* Infinite loop */
  for (;;)
  {
    uint8_t ReBuffer[8];
    BaseType_t xStatues;
		//float speed_;

    xStatues = xQueueReceive(QueueHandler, ReBuffer, portMAX_DELAY);
    if (xStatues == pdTRUE)
    {			
			//feedbackValue1 = (ReBuffer[0] << 8)  | ReBuffer[1];
			printf("%f,%f\n", outerTarget, (float)ReBuffer[0]);  
    }
    vTaskDelay(5);
  }
  /* USER CODE END StartTask03 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
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

#ifdef USE_FULL_ASSERT
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
