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
#include "DR16_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
QueueHandle_t QueueHandler;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for yaw */
osThreadId_t yawHandle;
const osThreadAttr_t yaw_attributes = {
    .name = "yaw",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void yaw_control(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  QueueHandler = xQueueCreate(20, 20);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of yaw */
  yawHandle = osThreadNew(yaw_control, NULL, &yaw_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint32_t tick_1 = 0;
  uint32_t tick_2 = 0;
  uint32_t tick_interval = 0;
  /* Infinite loop */
  for (;;)
  {
    tick_2 = HAL_GetTick(); // 获取当前时刻的tick值
    if (xSemaphoreTake(myBinarySem01Handle, 100) == pdTRUE)
      tick_1 = HAL_GetTick();        // 再次获取
    tick_interval = tick_1 - tick_2; // 判断两次tick之间的时间差

    if (tick_interval <= 100) // 如果小于等于100，说明在等待时间内获取了信号量
    {
      RemoteDataProcess((uint8_t *)RxBuffer);
    }
    else // 如果大于100，说明为超时退出
    {
      memset(&RC_CtrlData, 0, sizeof(RC_CtrlData));
    }
    xQueueSend(QueueHandler, &RC_CtrlData, 0);

    vTaskDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_yaw_control */
/**
 * @brief Function implementing the yaw thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_yaw_control */
void yaw_control(void *argument)
{
  /* USER CODE BEGIN yaw_control */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                // 使能中断
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)RxBuffer, LENGTH); // �?启DMA中断

  uint32_t TxID = 0x1FF;
  uint8_t TxLength = 8;
  uint8_t TxData[8] = {0};
  uint8_t TxData_1[8] = {0};

  HAL_CAN_Start(&hcan1);

  FilterInit();

  PID_Init(&mypid, 3, 1, 5, 20000, 15000);

  float prev_ema = 0; // 初始 EMA �?
  /* Infinite loop */
  for (;;)
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
  /* USER CODE END yaw_control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
