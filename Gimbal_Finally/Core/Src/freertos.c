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
#include "string.h"
#include "stdio.h"

// 自己移植的函�????????
#include "mycan.h"
#include "PID.h"
#include "DR16_control.h"
#include "Calculation.h"

// 外设
#include "usart.h"
#include "can.h"

// freertos
#include "queue.h"

// mpu
#include "MPU6050.h"
#include "inv_mpu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGTH 100 // 宏定�????????
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// mpu
float mpu_pitch, mpu_roll, mpu_yaw;
float mpu_gx, mpu_gy, mpu_gz;

// DMA
uint8_t RxBuffer[LENGTH];
uint8_t RecCount = 0;
uint8_t RxFlag = 0;

// yaw
typedef struct
{
  //  yaw_pid
  CascadePID pid_two;
  // yaw_Tx
  uint32_t TxID;
  uint8_t TxLength;
  uint8_t TxData[8];
  // yaw_pid
  float feedbackValue;
  float feedbackValue1;
  float targetValue;
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;
  // yaw_Rx
  uint32_t RxID;
  uint8_t RxLength;
  uint8_t RxData[8];
  // yaw_Measurement
  uint16_t encoder;
  float Angle;
  // yaw_can_return_flag
  uint8_t flag;
} YawStructDef;
YawStructDef YawStruct = {0};

// FrictionWheel
typedef struct
{
  // FrictionWheel_pid_L
  PID pid_one_L;
  // FrictionWheel_Tx_L
  uint32_t TxID_L;
  uint8_t TxLength_L;
  uint8_t TxData_L[8];
  // FrictionWheel_pid_L
  float feedbackValue_L;
  float feedbackValue1_L;
  float targetValue_L;
  int16_t Speed_L;
  // FrictionWheel_Rx_L
  uint32_t RxID_L; // 接受�????
  uint8_t RxLength_L;
  uint8_t RxData_L[8];
  // FrictionWheel_can_return_flag_L
  uint8_t flag_L;
  // FrictionWheel_pid_R
  PID pid_one_R;
  // FrictionWheel_Tx_R
  uint32_t TxID_R;
  uint8_t TxLength_R;
  uint8_t TxData_R[8];
  // FrictionWheel_pid_R
  float feedbackValue_R;
  float feedbackValue1_R;
  float targetValue_R;
  int16_t Speed_R;
  // FrictionWheel_Rx_R
  uint32_t RxID_R; // 接受�????
  uint8_t RxLength_R;
  uint8_t RxData_R[8];
  // FrictionWheel_can_return_flag_R
  uint8_t flag_R;
} FWStructDef;
FWStructDef FWStruct = {0};

// pitch
typedef struct
{
  // pitch_pid
  CascadePID pid_two;
  // pitch_Tx
  uint32_t TxID;
  uint8_t TxLength;
  uint8_t TxData[8];
  // pitch_pid
  float feedbackValue;
  float feedbackValue1;
  float targetValue;
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;
  // pitch_Rx
  uint32_t RxID; // 接受�????
  uint8_t RxLength;
  uint8_t RxData[8];
  // pitch_Measurement
  uint16_t encoder;
  float Angle;
  // pitch_can_return_flag_L
  uint8_t flag;
} pitchStructDef;
pitchStructDef pitchStruct = {0};

// dial
typedef struct
{
  // dial_pid
  CascadePID pid_two;
  // dial_Tx
  uint32_t TxID;
  uint8_t TxLength;
  uint8_t TxData[8];
  // dial_pid
  float feedbackValue;
  float feedbackValue1;
  float targetValue;
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;
  // dial_Rx
  uint32_t RxID; // 接受�????
  uint8_t RxLength;
  uint8_t RxData[8];
  // dial_Measurement
  uint16_t encoder;
  float Angle;
  // dial_can_return_flag_L
  uint8_t flag;
} dialStructDef;
dialStructDef dialStruct = {0};

// 平滑因子
float alpha = 0.05;

// 队列
QueueHandle_t QueueSemaYawHandler;
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
osSemaphoreId Semaphone_ProtectionHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CAN_Transmit(void const * argument);
void Semaphore(void const * argument);
void PID_Calculation(void const * argument);
void Mpu_Get(void const * argument);
void Print(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
void MX_FREERTOS_Init(void) {
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
  QueueSemaYawHandler = xQueueCreate(20, 20);
  QueueprintHandler = xQueueCreate(8, 4); // 创建队列
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CAN_Transmit_Ta */
  osThreadDef(CAN_Transmit_Ta, CAN_Transmit, osPriorityNormal, 0, 128);
  CAN_Transmit_TaHandle = osThreadCreate(osThread(CAN_Transmit_Ta), NULL);

  /* definition and creation of Semaphore_Task */
  osThreadDef(Semaphore_Task, Semaphore, osPriorityIdle, 0, 128);
  Semaphore_TaskHandle = osThreadCreate(osThread(Semaphore_Task), NULL);

  /* definition and creation of PID_Calculate_T */
  osThreadDef(PID_Calculate_T, PID_Calculation, osPriorityIdle, 0, 1280);
  PID_Calculate_THandle = osThreadCreate(osThread(PID_Calculate_T), NULL);

  /* definition and creation of Mpu_Task */
  osThreadDef(Mpu_Task, Mpu_Get, osPriorityIdle, 0, 128);
  Mpu_TaskHandle = osThreadCreate(osThread(Mpu_Task), NULL);

  /* definition and creation of Print_Task */
  osThreadDef(Print_Task, Print, osPriorityIdle, 0, 128);
  Print_TaskHandle = osThreadCreate(osThread(Print_Task), NULL);

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
void CAN_Transmit(void const * argument)
{
  /* USER CODE BEGIN CAN_Transmit */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // 定义类型，使用vTaskDelayUntil

  // 摩擦�???
  // 左轮
  FWStruct.TxID_L = 0x200;
  FWStruct.TxLength_L = 8;
  FWStruct.RxLength_L = 8;
  // 右轮
  FWStruct.TxID_R = 0x200;
  FWStruct.TxLength_R = 8;
  FWStruct.RxLength_R = 8;

  // 拨盘
  dialStruct.TxID = 0x200;
  dialStruct.TxLength = 8;
  dialStruct.RxLength = 8;

  // yaw
  YawStruct.TxID = 0x2FF;
  YawStruct.TxLength = 8;
  YawStruct.RxLength = 8;

  // pitch
  pitchStruct.TxID = 0x1FF;
  pitchStruct.TxLength = 8;
  pitchStruct.RxLength = 8;

  /* Infinite loop */
  for (;;)
  {

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
void Semaphore(void const * argument)
{
  /* USER CODE BEGIN Semaphore */
  uint32_t tick_1 = 0;
  uint32_t tick_2 = 0;
  uint32_t tick_interval = 0;
  /* Infinite loop */
  for (;;)
  {
    tick_2 = HAL_GetTick(); // 获取当前时刻的tick�????????????
    if (xSemaphoreTake(Semaphone_ProtectionHandle, 100) == pdTRUE)
      tick_1 = HAL_GetTick();        // 再次获取
    tick_interval = tick_1 - tick_2; // 判断两次tick之间的时间差

    if (tick_interval <= 100) // 如果小于等于100，说明在等待时间内获取了信号�????????????
    {
      RemoteDataProcess((uint8_t *)RxBuffer);
    }
    else // 如果大于100，说明为超时�????????????�????????????
    {
      memset(&RC_CtrlData, 0, sizeof(RC_CtrlData));
    }
    xQueueSend(QueueSemaYawHandler, &RC_CtrlData, 0);

    vTaskDelay(1);
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
void PID_Calculation(void const * argument)
{
  /* USER CODE BEGIN PID_Calculation */

  // 摩擦轮pid初始�???
  PID_Init(&FWStruct.pid_one_L, 0, 0, 0, 0, 0); // 左轮
  PID_Init(&FWStruct.pid_one_R, 0, 0, 0, 0, 0); // 右轮

  // 拨盘初始�??
  PID_Init(&dialStruct.pid_two.inner, 0, 0, 0, 0, 0); // 初始化内环参�???
  PID_Init(&dialStruct.pid_two.outer, 0, 0, 0, 0, 0); // 初始化外环参�???

  // Yaw初始�??
  PID_Init(&YawStruct.pid_two.inner, 0, 0, 0, 0, 0); // 初始化内环参�???
  PID_Init(&YawStruct.pid_two.outer, 0, 0, 0, 0, 0); // 初始化外环参�???

  // Pitch初始�??
  PID_Init(&pitchStruct.pid_two.inner, 0, 0, 0, 0, 0); // 初始化内环参�???
  PID_Init(&pitchStruct.pid_two.outer, 0, 0, 0, 0, 0); // 初始化外环参�???

  /* Infinite loop */
  for (;;)
  {
    //获取遥控值
    RemoteDataProcess(RxBuffer);
    Finally_Calculation(RC_CtrlData);
    //ch0输出值
    

    // 摩擦轮任务获取对应数值，并且进行pid计算得到Output
    // 左摩擦轮
    if (Can1_Receive_Judgment(0x203, FWStruct.RxData_L))
    {
      FWStruct.feedbackValue_L = PID_One_Calculation(&FWStruct.pid_one_L, FWStruct.targetValue_L, FWStruct.RxData_L);

      FWStruct.TxData_L[4] = (((int16_t)FWStruct.pid_one_L.output) >> 8) & 0xff; // 右移八位是因�????16位数据只有后面八位可以存�????8位的数组
      FWStruct.TxData_L[5] = ((int16_t)FWStruct.pid_one_L.output) & 0xff;
    }
    // 右摩擦轮
    if (Can1_Receive_Judgment(0x202, FWStruct.RxData_R))
    {
      FWStruct.feedbackValue_R = PID_One_Calculation(&FWStruct.pid_one_R, FWStruct.targetValue_R, FWStruct.RxData_R);

      FWStruct.TxData_R[2] = (((int16_t)FWStruct.pid_one_R.output) >> 8) & 0xff; // 右移八位是因�????16位数据只有后面八位可以存�????8位的数组
      FWStruct.TxData_R[3] = ((int16_t)FWStruct.pid_one_R.output) & 0xff;
    }

    // 拨盘任务获取对应数�?�，并且进行pid计算得到Output
    if (Can1_Receive_Judgment(0x201, dialStruct.RxData))
    {
      dialStruct.encoder = (dialStruct.RxData[0] << 8) | dialStruct.RxData[1];
      dialStruct.Angle = dialStruct.encoder * 360.0f / 8192.0f;
      dialStruct.Speed = (dialStruct.RxData[2] << 8) | dialStruct.RxData[3];

      dialStruct.outerFeedback = dialStruct.Angle;
      dialStruct.innerFeedback = dialStruct.Speed;                                                                                      // 获取内环反馈�???
      PID_CascadeCalc(&dialStruct.pid_two, dialStruct.outerTarget, dialStruct.outerFeedback, dialStruct.innerFeedback); // 进行PID计算

      dialStruct.TxData[0] = (((int16_t)dialStruct.pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
      dialStruct.TxData[1] = ((int16_t)dialStruct.pid_two.output) & 0xff;
    }

    // yaw任务获取对应数�?�，并且进行pid计算得到Output
    if (Can2_Receive_Judgment(0x209, YawStruct.RxData))
    {
      YawStruct.encoder = (YawStruct.RxData[0] << 8) | YawStruct.RxData[1];
      YawStruct.Angle = YawStruct.encoder * 360.0f / 8192.0f;
      YawStruct.Speed = (YawStruct.RxData[2] << 8) | YawStruct.RxData[3];

      YawStruct.outerFeedback = YawStruct.Angle;
      YawStruct.innerFeedback = YawStruct.Speed;                                                                    // 获取内环反馈�???
      PID_CascadeCalc(&YawStruct.pid_two, YawStruct.outerTarget, YawStruct.outerFeedback, YawStruct.innerFeedback); // 进行PID计算

      YawStruct.TxData[0] = (((int16_t)YawStruct.pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
      YawStruct.TxData[1] = ((int16_t)YawStruct.pid_two.output) & 0xff;
    }

    // pitch任务获取对应数�?�，并且进行pid计算得到Output
    if (Can1_Receive_Judgment(0x206, pitchStruct.RxData))
    {
      pitchStruct.encoder = (pitchStruct.RxData[0] << 8) | pitchStruct.RxData[1];
      pitchStruct.Angle = pitchStruct.encoder * 360.0f / 8192.0f;
      pitchStruct.Speed = (pitchStruct.RxData[2] << 8) | pitchStruct.RxData[3];

      pitchStruct.outerFeedback = pitchStruct.Angle;
      pitchStruct.innerFeedback = pitchStruct.Speed;                                                                        // 获取内环反馈�???
      PID_CascadeCalc(&pitchStruct.pid_two, pitchStruct.outerTarget, pitchStruct.outerFeedback, pitchStruct.innerFeedback); // 进行PID计算

      pitchStruct.TxData[2] = (((int16_t)pitchStruct.pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
      pitchStruct.TxData[3] = ((int16_t)pitchStruct.pid_two.output) & 0xff;
    }

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
void Mpu_Get(void const * argument)
{
  /* USER CODE BEGIN Mpu_Get */
  /* Infinite loop */
  for (;;)
  {
    mpu_dmp_get_data(&mpu_pitch, &mpu_roll, &mpu_yaw);
    MPU_Get_Gyroscope(&mpu_gx, &mpu_gy, &mpu_gz);
    vTaskDelay(1);
  }
  /* USER CODE END Mpu_Get */
}

/* USER CODE BEGIN Header_Print */
/**
 * @brief Function implementing the Print_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Print */
void Print(void const * argument)
{
  /* USER CODE BEGIN Print */
  /* Infinite loop */
  for (;;)
  {
    vTaskDelay(1);
  }
  /* USER CODE END Print */
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
  //	xSemaphoreGive(myBinarySem01Handle);

  RxFlag = 1;
  HAL_UART_DMAStop(&huart2); // 关闭DMA，每次空闲都会进函数

  if (RxFlag == 1)
  {
    RxFlag = 0;
    RecCount = LENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    //	printf("%s\r\n",RxBuffer);
    RecCount = 0;
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)RxBuffer, LENGTH);
  }
}

int fputc(int ch, FILE *f) // 重定向fputc函数
{
  HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int fgetc(FILE *f) // 重定向fgetc函数
{
  uint8_t ch;
  HAL_UART_Receive(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END Application */
