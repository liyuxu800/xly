/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_drive.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// dma接收长度
#define LENGTH 100

// can发�?�长�?????
#define Transmit_Length 8

// 摩擦�?????
//  左轮
#define FWStruct_TxID_L 0x200
// 右轮
#define FWStruct_TxID_R 0x200

// 拨盘
#define dialStruct_TxID 0x200

// yaw
#define YawStruct_TxID 0x2FF

// pitch
#define pitchStruct_TxID 0x1FF

// 摩擦轮�?�拨�?
#define FW_dial_TxID 0x200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

typedef struct
{
  float DataBuff[3];
  uint8_t tail[4];
} DataBuffStructDef;
DataBuffStructDef DataBuffStruct = {0};

// DMA
uint8_t RxBuffer[LENGTH];
uint8_t RecCount = 0;
uint8_t RxFlag = 0;

// 输出保护变量
uint8_t output_protection = 0;

// justfloat打印定义数组
// uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

// 队列
QueueHandle_t QueueprintHandler;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId CAN_Transmit_TaHandle;
osThreadId Semaphore_TaskHandle;
osThreadId PID_Calculate_THandle;
osThreadId Mpu_TaskHandle;
osThreadId Print_TaskHandle;
osThreadId State_Machine_THandle;
osSemaphoreId Semaphone_ProtectionHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CAN_Transmit(void const *argument);
void Semaphore(void const *argument);
void PID_Calculation(void const *argument);
void Mpu_Get(void const *argument);
void Print(void const *argument);
void State_Machine(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of Semaphone_Protection */
  osSemaphoreDef(Semaphone_Protection);
  Semaphone_ProtectionHandle = osSemaphoreCreate(osSemaphore(Semaphone_Protection), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  QueueprintHandler = xQueueCreate(8, 4); // 创建队列
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CAN_Transmit_Ta */
  osThreadDef(CAN_Transmit_Ta, CAN_Transmit, osPriorityNormal, 0, 256);
  CAN_Transmit_TaHandle = osThreadCreate(osThread(CAN_Transmit_Ta), NULL);

  /* definition and creation of Semaphore_Task */
  osThreadDef(Semaphore_Task, Semaphore, osPriorityIdle, 0, 128);
  Semaphore_TaskHandle = osThreadCreate(osThread(Semaphore_Task), NULL);

  /* definition and creation of PID_Calculate_T */
  osThreadDef(PID_Calculate_T, PID_Calculation, osPriorityIdle, 0, 1280);
  PID_Calculate_THandle = osThreadCreate(osThread(PID_Calculate_T), NULL);

  /* definition and creation of Mpu_Task */
  osThreadDef(Mpu_Task, Mpu_Get, osPriorityNormal, 0, 128);
  Mpu_TaskHandle = osThreadCreate(osThread(Mpu_Task), NULL);

  /* definition and creation of Print_Task */
  osThreadDef(Print_Task, Print, osPriorityIdle, 0, 128);
  Print_TaskHandle = osThreadCreate(osThread(Print_Task), NULL);

  /* definition and creation of State_Machine_T */
  osThreadDef(State_Machine_T, State_Machine, osPriorityIdle, 0, 128);
  State_Machine_THandle = osThreadCreate(osThread(State_Machine_T), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_CAN_Transmit */
/**
 * @brief  Function implementing the CAN_Transmit_Ta thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_CAN_Transmit */
void CAN_Transmit(void const *argument)
{
  /* USER CODE BEGIN CAN_Transmit */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // 定义类型，使用vTaskDelayUntil

  /* Infinite loop */
  for (;;)
  {

    //      		//发�?�左摩擦�??
    //          CAN1_Transmit(FWStruct_TxID_L, Transmit_Length, FWStruct.TxData_L);
    //    		//发�?�右摩擦�??
    //       CAN1_Transmit(FWStruct_TxID_R, Transmit_Length, FWStruct.TxData_R);
    //    // 发�?�dial
    //    CAN1_Transmit(dialStruct_TxID, Transmit_Length, dialStruct.TxData);
    //        		//yaw
    //            CAN2_Transmit(YawStruct_TxID, Transmit_Length, YawStruct.TxData);
    // 发�?�pitch
    CAN1_Transmit(pitchStruct_TxID, Transmit_Length, pitchStruct.TxData);
    //

    // CAN整体发�??
    // CAN1_Transmit(FW_dial_TxID, Transmit_Length, FW_dial_TxData);
    // CAN1_Transmit(pitchStruct_TxID, Transmit_Length, FW_dial_TxData);
    // CAN2_Transmit(YawStruct_TxID, Transmit_Length, FW_dial_TxData);

    vTaskDelayUntil(&xLastWakeTime, 1); // 延时
  }
  /* USER CODE END CAN_Transmit */
}

/* USER CODE BEGIN Header_Semaphore */
/**
 * @brief Function implementing the Semaphore_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Semaphore */
void Semaphore(void const *argument)
{
  /* USER CODE BEGIN Semaphore */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                // 使能中断
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)RxBuffer, LENGTH); // 弿启DMA中断

  uint32_t tick_1 = 0;
  uint32_t tick_2 = 0;
  uint32_t tick_interval = 0;
  /* Infinite loop */
  for (;;)
  {
    tick_2 = HAL_GetTick(); // 获取当前时刻的tick�??????????????????
    if (xSemaphoreTake(Semaphone_ProtectionHandle, 100) == pdTRUE)
      tick_1 = HAL_GetTick();        // 再次获取
    tick_interval = tick_1 - tick_2; // 判断两次tick之间的时间差

    if (tick_interval < 100) // 如果小于100，说明在等待时间内获取了信号�??????????????????
    {
      // 获取遥控值并处理
      RemoteDataProcess(RxBuffer);
      output_protection = 1;
    }
    else
    // 如果大于等于100，说明为超时�??????????????????�??????????????????
    {
      memset(&RxBuffer, 0, sizeof(RxBuffer));
      output_protection = 0;
    }
    vTaskDelay(5);
  }
  /* USER CODE END Semaphore */
}

/* USER CODE BEGIN Header_PID_Calculation */
/**
 * @brief Function implementing the PID_Calculate_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PID_Calculation */
void PID_Calculation(void const *argument)
{
  /* USER CODE BEGIN PID_Calculation */

  // 注意把最大输出设置为0了，防止突然启动造成意外

  // Yaw初始�????????
  PID_Init(&YawStruct.pid_two.inner, 2000, 1000, 0, 0.0001, 0); // 初始化内环参�?????????
  PID_Init(&YawStruct.pid_two.outer, 2, 0, 0, 0, 0);            // 初始化外环参�?????????

  // Pitch初始�????????
  PID_Init(&pitchStruct.pid_two.inner, 700, 0, 0, 0, 25000); // 初始化内环参�?????????
  PID_Init(&pitchStruct.pid_two.outer, 13, 0, 0, 0, 25000);  // 初始化外环参�?????????

  // 摩擦轮pid初始�?????????
  PID_Init(&FWStruct.pid_one_L, 10, 0, 0, 0, 25000); // 左轮
  PID_Init(&FWStruct.pid_one_R, 10, 0, 0, 0, 25000); // 右轮

  // 拨盘初始�????????
  PID_Init(&dialStruct.pid_two.inner, 120, 0, 0, 0, 25000); // 初始化内环参�?????????
  PID_Init(&dialStruct.pid_two.outer, 9, 0, 45, 0, 25000);  // 初始化外环参�?????????

  HAL_CAN_Start(&hcan1); // 启动CAN
  HAL_CAN_Start(&hcan2);

  FilterInit(); // 配置过滤�??????

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能can接收中断
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能can接收中断

  /* Infinite loop */
  for (;;)
  {

    // 获取数�?�进行PID计算，并且进行数值位运算

    Yaw_Motor_Control(&YawStruct);
    Pitch_Motor_Control(&pitchStruct);
    FW_Motor_Control(&FWStruct);

    Dial_Motor_Control(&dialStruct);

    // 打印摩擦�??
    //    		//左轮
    //    		DataBuffStruct.DataBuff[0] = FWStruct.feedbackValue_L;
    //       DataBuffStruct.DataBuff[1] = FWStruct.targetValue_L;
    //       DataBuffStruct.DataBuff[2] = FWStruct.pid_one_L.output;
    //    		//右轮
    //    		DataBuffStruct.DataBuff[0] = FWStruct.feedbackValue_R;
    //        DataBuffStruct.DataBuff[1] = FWStruct.targetValue_R;
    //        DataBuffStruct.DataBuff[2] = FWStruct.pid_one_R.output;
    taskENTER_CRITICAL();
    //	    		//打印dial
    //	DataBuffStruct.DataBuff[0] = dialStruct.outerFeedback;
    //	DataBuffStruct.DataBuff[1] = dialStruct.outerTarget;
    //	DataBuffStruct.DataBuff[2] = dialStruct.pid_two.output;
    //
    //

    //
    //        		//打印yaw
    //						DataBuffStruct.DataBuff[0] = YawStruct.outerFeedback;
    //        		DataBuffStruct.DataBuff[0] = YawStruct.Angle;
    //            DataBuffStruct.DataBuff[1] = YawStruct.outerTarget;
    //            DataBuffStruct.DataBuff[2] = YawStruct.pid_two.output;
    //
    //    // 打印pitch
    DataBuffStruct.DataBuff[0] = pitchStruct.Angle;
    DataBuffStruct.DataBuff[1] = pitchStruct.outerTarget;
    DataBuffStruct.DataBuff[2] = pitchStruct.pid_two.output;

    //    xQueueSend(QueueprintHandler, (uint8_t *)&DataBuffStruct, 0);
    taskEXIT_CRITICAL();
    vTaskDelay(1);
  }
  /* USER CODE END PID_Calculation */
}

/* USER CODE BEGIN Header_Mpu_Get */
/**
 * @brief Function implementing the Mpu_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Mpu_Get */
void Mpu_Get(void const *argument)
{
  /* USER CODE BEGIN Mpu_Get */
  MPU_Init();
  mpu_dmp_init();
  while (mpu_dmp_init())
  {
    MPU_Init();
    mpu_dmp_init();
  }
  /* Infinite loop */
  for (;;)
  {
    MPU_Get_Gyroscope(&gx, &gy, &gz);
    mpu_dmp_get_data(&mpu_pitch, &mpu_roll, &mpu_yaw);
    vTaskDelay(1);
  }

  // ptch轴为车的row�??
  // row轴为车的pitch轴，�??上面�??-22，最下面�??23      刻度值为2445 �??   1385
  // yaw轴对应车的yao的yaw轴，0�??180，突变为-180，再�??-0
  // gx对应pitch
  // gz对应yaw

  /* USER CODE END Mpu_Get */
}

/* USER CODE BEGIN Header_Print */
/**
 * @brief Function implementing the Print_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Print */
void Print(void const *argument)
{
  /* USER CODE BEGIN Print */

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // 定义类型，使用vTaskDelayUntil

  /* Infinite loop */
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, 2);

    DataBuffStruct.tail[0] = 0x00;
    DataBuffStruct.tail[1] = 0x00;
    DataBuffStruct.tail[2] = 0x80;
    DataBuffStruct.tail[3] = 0x7F;

    // justfloat打印
    HAL_UART_Transmit(&huart5, (uint8_t *)&DataBuffStruct, sizeof(DataBuffStruct), HAL_MAX_DELAY);
  }
  /* USER CODE END Print */
}

/* USER CODE BEGIN Header_State_Machine */
/**
 * @brief Function implementing the State_Machine_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_State_Machine */
void State_Machine(void const *argument)
{
  /* USER CODE BEGIN State_Machine */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // 定义类型，使用vTaskDelayUntil

  static uint8_t flag = 0;

  /* Infinite loop */
  for (;;)
  {
    if (RC_CtrlData.rc.s2 == 3 && flag == 2)
    {
      flag = 0;
    }
    else if (RC_CtrlData.rc.s2 == 1 && flag == 0)
    {
      flag = 1;
      if (RC_CtrlData.rc.s2 == 1 && flag == 1)
      {
        dialStruct.outerTarget = dialStruct.outerFeedback + 40;
        flag = 2;
      }
    }
    else if (RC_CtrlData.rc.s2 == 2)
    {
      dialStruct.outerTarget = dialStruct.outerFeedback + 40;
    }

    vTaskDelayUntil(&xLastWakeTime, 50); // 延时
  }
  /* USER CODE END State_Machine */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 串口中断回调函数
{
  if (huart->Instance == USART2)
  {
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)RxBuffer, LENGTH);
  }
}

void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart) // 空闲中断回调函数
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(Semaphone_ProtectionHandle, &xHigherPriorityTaskWoken);

  RxFlag = 1;
  HAL_UART_DMAStop(&huart2); // 关闭DMA，每次空闲都会进函数

  if (RxFlag == 1)
  {
    RxFlag = 0;
    RecCount = LENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    RecCount = 0;
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)RxBuffer, LENGTH);
  }
}
/* USER CODE END Application */
