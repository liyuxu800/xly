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

// 自己移植的函�???
#include "mycan.h"
#include "PID.h"
#include "DR16_control.h"

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
#define LENGTH 100 // 宏定
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
//  yaw_pid
PID pid_yaw_one = {0};
// yaw_Tx
uint32_t TxID_yaw = 0x2FF;
uint8_t TxLength_yaw = 8;
uint8_t TxData_yaw[8] = {0};
// yaw_pid
float feedbackValue_yaw;
float feedbackValue1_yaw;
float targetValue_yaw = 0;
int16_t Speed_yaw;
// yaw_Rx
uint32_t RxID_yaw = {0};
uint8_t RxLength_yaw = 8;
uint8_t RxData_yaw[8];
// yaw_RC_CtrlData
RC_Ctl_t RC_CtrlData_yaw;

// FrictionWheel_pid_L
PID pid_FrictionWheel_one_L = {0};
// FrictionWheel_Tx_L
uint32_t TxID_FrictionWheel_L = 0x200;
uint8_t TxLength_FrictionWheel_L = 8;
uint8_t TxData_FrictionWheel_L[8] = {0};
// FrictionWheel_pid_L
float feedbackValue_FrictionWheel_L;
float feedbackValue1_FrictionWheel_L;
float targetValue_FrictionWheel_L = 0;
int16_t Speed_FrictionWheel_L;
// FrictionWheel_Rx_L
uint32_t RxID_FrictionWheel_L; // 接受�?
uint8_t RxLength_FrictionWheel_L = 8;
uint8_t RxData_FrictionWheel_L[8];
// FrictionWheel_pid_R
PID pid_FrictionWheel_one_R = {0};
// FrictionWheel_Tx_R
uint32_t TxID_FrictionWheel_R = 0x200;
uint8_t TxLength_FrictionWheel_R = 8;
uint8_t TxData_FrictionWheel_R[8] = {0};
// FrictionWheel_pid_R
float feedbackValue_FrictionWheel_R;
float feedbackValue1_FrictionWheel_R;
float targetValue_FrictionWheel_R = 0;
int16_t Speed_FrictionWheel_R;
// FrictionWheel_Rx_R
uint32_t RxID_FrictionWheel_R; // 接受�?
uint8_t RxLength_FrictionWheel_R = 8;
uint8_t RxData_FrictionWheel_R[8];

// pitch
typedef struct
{
  // pitch_pid
  CascadePID pid_two;
  // pitch_Tx
  uint32_t TxID;
  uint8_t TxLength;
  uint8_t TxDatal[8];
  // pitch_pid
  float feedbackValue;
  float feedbackValue1;
  float targetValue;
  int16_t Speed;
  // pitch_Rx
  uint32_t RxID; // 接受�?
  uint8_t RxLength;
  uint8_t RxData[8];
  // pitch_Measurement
  uint16_t encoder;
  float Angle;
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
  uint8_t TxDatal[8];
  // dial_pid
  float feedbackValue;
  float feedbackValue1;
  float targetValue;
  int16_t Speed;
  // dial_Rx
  uint32_t RxID; // 接受�?
  uint8_t RxLength;
  uint8_t RxData[8];
  // dial_Measurement
  uint16_t encoder;
  float Angle;
} dialStructDef;
dialStructDef dialStruct = {0};

// 平滑因子
float alpha = 0.05;

// 队列
QueueHandle_t QueueSemaYawHandler;
QueueHandle_t QueueprintHandler;
// QueueHandle_t QueueMpuYawHandler;
// QueueHandle_t QueueMpuPitchHandler;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Semaphore_TaskHandle;
osThreadId Yaw_Control_TasHandle;
osThreadId Print_TaskHandle;
osThreadId pitch_taskHandle;
osThreadId Mpu_Get_TaskHandle;
osThreadId FrictionWheel_THandle;
osThreadId Dial_TaskHandle;
osSemaphoreId myBinarySem01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Semaphore(void const *argument);
void yaw_control(void const *argument);
void print_task(void const *argument);
void pitch_control(void const *argument);
void Mpu_Get(void const *argument);
void FrictionWheel_Control(void const *argument);
void dial_control(void const *argument);

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
  QueueSemaYawHandler = xQueueCreate(20, 20);
  QueueprintHandler = xQueueCreate(8, 4); // 创建队列
  // QueueMpuYawHandler = xQueueCreate(4, 4);
  // QueueMpuPitchHandler = xQueueCreate(4, 4);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Semaphore_Task */
  osThreadDef(Semaphore_Task, Semaphore, osPriorityBelowNormal, 0, 256);
  Semaphore_TaskHandle = osThreadCreate(osThread(Semaphore_Task), NULL);

  /* definition and creation of Yaw_Control_Tas */
  osThreadDef(Yaw_Control_Tas, yaw_control, osPriorityNormal, 0, 256);
  Yaw_Control_TasHandle = osThreadCreate(osThread(Yaw_Control_Tas), NULL);

  /* definition and creation of Print_Task */
  osThreadDef(Print_Task, print_task, osPriorityIdle, 0, 128);
  Print_TaskHandle = osThreadCreate(osThread(Print_Task), NULL);

  /* definition and creation of pitch_task */
  osThreadDef(pitch_task, pitch_control, osPriorityIdle, 0, 256);
  pitch_taskHandle = osThreadCreate(osThread(pitch_task), NULL);

  /* definition and creation of Mpu_Get_Task */
  osThreadDef(Mpu_Get_Task, Mpu_Get, osPriorityIdle, 0, 128);
  Mpu_Get_TaskHandle = osThreadCreate(osThread(Mpu_Get_Task), NULL);

  /* definition and creation of FrictionWheel_T */
  osThreadDef(FrictionWheel_T, FrictionWheel_Control, osPriorityIdle, 0, 128);
  FrictionWheel_THandle = osThreadCreate(osThread(FrictionWheel_T), NULL);

  /* definition and creation of Dial_Task */
  osThreadDef(Dial_Task, dial_control, osPriorityIdle, 0, 128);
  Dial_TaskHandle = osThreadCreate(osThread(Dial_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_Semaphore */
/**
 * @brief  Function implementing the Semaphore_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Semaphore */
void Semaphore(void const *argument)
{
  /* USER CODE BEGIN Semaphore */
  uint32_t tick_1 = 0;
  uint32_t tick_2 = 0;
  uint32_t tick_interval = 0;
  /* Infinite loop */
  for (;;)
  {
    tick_2 = HAL_GetTick(); // 获取当前时刻的tick�???????
    if (xSemaphoreTake(myBinarySem01Handle, 100) == pdTRUE)
      tick_1 = HAL_GetTick();        // 再次获取
    tick_interval = tick_1 - tick_2; // 判断两次tick之间的时间差

    if (tick_interval <= 100) // 如果小于等于100，说明在等待时间内获取了信号�???????
    {
      RemoteDataProcess((uint8_t *)RxBuffer);
    }
    else // 如果大于100，说明为超时�???????�???????
    {
      memset(&RC_CtrlData, 0, sizeof(RC_CtrlData));
    }
    xQueueSend(QueueSemaYawHandler, &RC_CtrlData, 0);

    vTaskDelay(1);
  }
  /* USER CODE END Semaphore */
}

/* USER CODE BEGIN Header_yaw_control */
/**
 * @brief Function implementing the Yaw_Control_Tas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_yaw_control */
void yaw_control(void const *argument)
{
  /* USER CODE BEGIN yaw_control */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  PID_Init(&pid_yaw_one, 3, 1, 5, 500, 800); // 初始化pid

  float prev_ema = 0; // 初始 EMA �???????
  /* Infinite loop */
  for (;;)
  {
    xQueueReceive(QueueSemaYawHandler, &RC_CtrlData_yaw, 0); // 接收来自信号量任务的目标值
    // xQueueReceive(QueueMpuYawHandler, (uint8_t *)&yaw_Queue_Receive, 0);    //当前的yaw值

    targetValue_yaw = RC_CtrlData_yaw.rc.ch0; // 遥控值等于目标�??

    Speed_yaw = (RxData[2] << 8) | RxData[3]; // 读取实际速度

    feedbackValue_yaw = Speed_yaw; // 这里获取到被控对象的反馈�???????

    float ema_result = emaFilter(feedbackValue_yaw, &prev_ema, alpha);

    PID_Calc(&pid_yaw_one, targetValue_yaw, ema_result); // 进行PID计算，结果在output成员变量

    TxData_yaw[0] = (((int16_t)pid_yaw_one.output) >> 8) & 0xff; // 右移八位是因�???????16位数据只有后面八位可以存�???????8位的数组
    TxData_yaw[1] = ((int16_t)pid_yaw_one.output) & 0xff;

    CAN2_Transmit(TxID_yaw, TxLength_yaw, TxData_yaw);

    // printf("%f,%f,%f\n", targetValue, ema_result,mypid.output);
    xQueueSend(QueueprintHandler, (uint8_t *)(void *)(&ema_result), 0);

    vTaskDelayUntil(&xLastWakeTime, 1);
  }
  /* USER CODE END yaw_control */
}

/* USER CODE BEGIN Header_print_task */
/**
 * @brief Function implementing the Print_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_print_task */
void print_task(void const *argument)
{
  /* USER CODE BEGIN print_task */
  /* Infinite loop */
  for (;;)
  {
    BaseType_t xStatues;
    float speed_;

    xStatues = xQueueReceive(QueueprintHandler, (uint8_t *)(void *)&speed_, portMAX_DELAY); // 接收数据
    if (xStatues == pdTRUE)
    {
      printf("%f,%f,%f\n", targetValue_yaw, speed_, pid_yaw_one.output);
    }
    vTaskDelay(3);
  }
  /* USER CODE END print_task */
}

/* USER CODE BEGIN Header_pitch_control */
/**
 * @brief Function implementing the pitch_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_pitch_control */
void pitch_control(void const *argument)
{
  /* USER CODE BEGIN pitch_control */
  pitchStruct.TxID = 0x1FF; // 发送ID
  pitchStruct.TxLength = 8;
  pitchStruct.RxLength = 8;

  // xQueueReceive(QueueMpuPitchHandler, (uint8_t *)&pitchStruct.Pitch_Queue_Receive, 0); // 接收pitch值队列

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // 定义类型，使用vTaskDelayUntil

  PID_Init(&pitchStruct.pid_two.inner, 30, 0, 0, 0, 25000); // 初始化内环参�?
  PID_Init(&pitchStruct.pid_two.outer, 70, 0, 0, 0, 25000); // 初始化外环参�?

  /* Infinite loop */
  for (;;)
  {
    pitchStruct.encoder = (RxData[0] << 8) | RxData[1];
    pitchStruct.Angle = pitchStruct.encoder * 360.0f / 8192.0f;
    pitchStruct.Speed = (RxData[2] << 8) | RxData[3];

    pitchStruct.outerFeedback = pitchStruct.Angle;

    pitchStruct.innerFeedback = pitchStruct.Speed;                                                                        // 获取内环反馈�?
    PID_CascadeCalc(&pitchStruct.pid_two, pitchStruct.outerTarget, pitchStruct.outerFeedback, pitchStruct.innerFeedback); // 进行PID计算

    TxData[0] = (((int16_t)pid_two.output) >> 8) & 0xff; // 右移八位是因�?16位数据只有后面八位可以存�?8位的数组
    TxData[1] = ((int16_t)pid_two.output) & 0xff;

    CAN1_Transmit(pitchStruct.TxID, pitchStruct.TxLength, pitchStruct.TxData);

    // tx_data.data[0] = outerFeedback;
    // tx_data.data[1] = outerTarget;
    // tx_data.data[2] = pid_two.output / 100.0f;

    // xQueueSend(QueueHandler, (uint8_t *)&tx_data, 0);

    vTaskDelayUntil(&xLastWakeTime, 1); // 延时
  }
  /* USER CODE END pitch_control */
}

/* USER CODE BEGIN Header_Mpu_Get */
/**
 * @brief Function implementing the Mpu_Get_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Mpu_Get */
void Mpu_Get(void const *argument)
{
  /* USER CODE BEGIN Mpu_Get */
  /* Infinite loop */
  for (;;)
  {
    mpu_dmp_get_data(&mpu_pitch, &mpu_roll, &mpu_yaw);
    MPU_Get_Gyroscope(&mpu_gx, &mpu_gy, &mpu_gz);
    // printf("pitch %f, roll = %f, yaw = %f\r\n", mpu_pitch, mpu_roll, mpu_yaw);
    // xQueueSend(QueueMpuYawHandler, (uint8_t *)&MpuStruct.mpu_yaw, 0);
    // xQueueSend(QueueMpuPitchHandler, (uint8_t *)&MpuStruct.mpu_pitch, 0);
    vTaskDelay(1);
  }
  /* USER CODE END Mpu_Get */
}

/* USER CODE BEGIN Header_FrictionWheel_Control */
/**
 * @brief Function implementing the FrictionWheel_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_FrictionWheel_Control */
void FrictionWheel_Control(void const *argument)
{
  /* USER CODE BEGIN FrictionWheel_Control */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  PID_Init(&pid_FrictionWheel_one_L, 3, 1, 5, 20000, 15000);
  PID_Init(&pid_FrictionWheel_one_R, 3, 1, 5, 20000, 15000);
  /* Infinite loop */
  for (;;)
  {
    if (rceStu_can1_fifo0.StdId == 0x203)
    {
      for (uint8_t i = 0; i <= 8; i++)
      {
        RxData_FrictionWheel_L[i] = Data_can1_fifo0[i];
      }
    }
    if (rceStu_can1_fifo0.StdId == 0x202)
    {
      for (uint8_t i = 0; i <= 8; i++)
      {
        RxData_FrictionWheel_R[i] = Data_can1_fifo0[i];
      }
    }

    Speed_FrictionWheel_L = (RxData_FrictionWheel_L[2] << 8) | RxData_FrictionWheel_L[3]; // 左摩擦轮实际速度
    Speed_FrictionWheel_R = (RxData_FrictionWheel_R[2] << 8) | RxData_FrictionWheel_R[3]; // 右摩擦轮实际速度

    feedbackValue_FrictionWheel_L = Speed_FrictionWheel_L; // 这里获取到被控对象的反馈�?
    feedbackValue_FrictionWheel_R = Speed_FrictionWheel_R; // 这里获取到被控对象的反馈�?

    // float ema_result = emaFilter(feedbackValue_FrictionWheel, &prev_ema, alpha);

    PID_Calc(&pid_FrictionWheel_one_L, targetValue_FrictionWheel_L, feedbackValue_FrictionWheel_L); // 进行PID计算，结果在output成员变量
    PID_Calc(&pid_FrictionWheel_one_R, targetValue_FrictionWheel_R, feedbackValue_FrictionWheel_R); // 进行PID计算，结果在output成员变量

    TxData_FrictionWheel_L[4] = (((int16_t)pid_FrictionWheel_one_L.output) >> 8) & 0xff; // 右移八位是因�?16位数据只有后面八位可以存�?8位的数组
    TxData_FrictionWheel_L[5] = ((int16_t)pid_FrictionWheel_one_L.output) & 0xff;
    TxData_FrictionWheel_R[2] = (((int16_t)pid_FrictionWheel_one_R.output) >> 8) & 0xff; // 右移八位是因�?16位数据只有后面八位可以存�?8位的数组
    TxData_FrictionWheel_R[3] = ((int16_t)pid_FrictionWheel_one_R.output) & 0xff;

    CAN1_Transmit(TxID_FrictionWheel_L, TxLength_FrictionWheel_L, TxData_FrictionWheel_L);
    CAN1_Transmit(TxID_FrictionWheel_R, TxLength_FrictionWheel_R, TxData_FrictionWheel_R);

    // xQueueSend(QueueHandler, (uint8_t *)(void *)(&ema_result), 0);

    vTaskDelayUntil(&xLastWakeTime, 1); // 延时
  }
  /* USER CODE END FrictionWheel_Control */
}

/* USER CODE BEGIN Header_dial_control */
/**
 * @brief Function implementing the Dial_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_dial_control */
void dial_control(void const *argument)
{
  /* USER CODE BEGIN dial_control */
  dialStruct.TxID = 0x1FF;
  dialStruct.TxLength = 8;
  dialStruct.RxLength = 8;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // 定义类型，使用vTaskDelayUntil

  PID_Init(&dialStruct.pid_two.inner, 30, 0, 0, 0, 25000); // 初始化内环参�?
  PID_Init(&dialStruct.pid_two.outer, 70, 0, 0, 0, 25000); // 初始化外环参�?
  /* Infinite loop */
  for (;;)
  {
    dialStruct.encoder = (RxData[0] << 8) | RxData[1];
    dialStruct.Angle = dialStruct.encoder * 360.0f / 8192.0f;
    dialStruct.Speed = (RxData[2] << 8) | RxData[3];

    dialStruct.outerFeedback = dialStruct.Angle;

    dialStruct.innerFeedback = dialStruct.Speed;                                                                      // 获取内环反馈�?
    PID_CascadeCalc(&dialStruct.pid_two, dialStruct.outerTarget, dialStruct.outerFeedback, dialStruct.innerFeedback); // 进行PID计算

    TxData[0] = (((int16_t)dialStruct.pid_two.output) >> 8) & 0xff; // 右移八位是因�?16位数据只有后面八位可以存�?8位的数组
    TxData[1] = ((int16_t)dialStruct.pid_two.output) & 0xff;

    CAN1_Transmit(dialStruct.TxID, dialStruct.TxLength, dialStruct.TxData);

    // tx_data.data[0] = outerFeedback;
    // tx_data.data[1] = outerTarget;
    // tx_data.data[2] = pid_two.output / 100.0f;

    // xQueueSend(QueueHandler, (uint8_t *)&tx_data, 0);

    vTaskDelayUntil(&xLastWakeTime, 1); // 延时
  }
  /* USER CODE END dial_control */
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

  xSemaphoreGiveFromISR(myBinarySem01Handle, &xHigherPriorityTaskWoken);
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
