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
#include "mycan.h"
#include "PID_Single_Loop.h"
#include "DR16_control.h"
#include "stdio.h"
#include "usart.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  LENGTH  18
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t RxBuffer[LENGTH];
uint8_t RecCount = 0;
uint8_t RxFlag = 0;	

float feedbackValue;
float feedbackValue1;
float targetValue = 0;

uint32_t RxID;
uint8_t RxLength = 8;
uint8_t RxData[8];

int16_t Speed;

float alpha = 0.05;  // 平滑因子

QueueHandle_t QueueHandler;
QueueHandle_t QueueprintHandler;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId controlHandle;
osThreadId Print_TaskHandle;
osSemaphoreId myBinarySem01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void DR16_control(void const * argument);
void print_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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
	QueueHandler = xQueueCreate(20, 20);
  if (QueueHandler == NULL)
  {
    printf("error");
  }
	QueueprintHandler = xQueueCreate(8, 4);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of control */
  osThreadDef(control, DR16_control, osPriorityIdle, 0, 256);
  controlHandle = osThreadCreate(osThread(control), NULL);

  /* definition and creation of Print_Task */
  osThreadDef(Print_Task, print_task, osPriorityIdle, 0, 128);
  Print_TaskHandle = osThreadCreate(osThread(Print_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint32_t tick_1 = 0;
	uint32_t tick_2 = 0;
	uint32_t tick_interval = 0;
  /* Infinite loop */
  for(;;)
  {
   		tick_2 = HAL_GetTick();		//获取当前时刻的tick�?
		if(xSemaphoreTake(myBinarySem01Handle,100) == pdTRUE)
		tick_1 = HAL_GetTick();			//再次获取
		tick_interval = tick_1 - tick_2;	//判断两次tick之间的时间差

			if(tick_interval <= 100)		//如果小于等于100，说明在等待时间内获取了信号�?
			{
				RemoteDataProcess((uint8_t*)RxBuffer);
			}
			else						//如果大于100，说明为超时�?�?
			{
				memset(&RC_CtrlData, 0, sizeof(RC_CtrlData));
			}
			xQueueSend(QueueHandler , &RC_CtrlData, 0);
	
    vTaskDelay(1);
  }
  /* USER CODE END StartDefaultTask */
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
  HAL_UART_Receive_DMA(&huart2,(uint8_t*)RxBuffer,LENGTH);		//弿启DMA中断
	
	static uint8_t i = 0;
	uint32_t TxID = 0x2FF;
  uint8_t TxLength = 8;
  uint8_t TxData[8] = {0};
	uint8_t TxData_1[8] = {0};

  HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);

  FilterInit();

  PID_Init(&mypid, 3, 1, 5, 10000, 15000);

  float prev_ema = 0; // 初始 EMA �?

  /* Infinite loop */
  for(;;)
  {
   xQueueReceive(QueueHandler, &RC_CtrlData, 0);
		
		TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
		
		targetValue = RC_CtrlData.rc.ch0 ;

    CAN2_Receive(&RxID, &RxLength, RxData);
    Speed = (RxData[2] << 8) | RxData[3];

    feedbackValue = Speed; // 这里获取到被控对象的反馈�?

    float ema_result = emaFilter(feedbackValue, &prev_ema, alpha);

    PID_Calc(&mypid, targetValue, ema_result); // 进行PID计算，结果在output成员变量

    TxData[0] = (((int16_t)mypid.output) >> 8) & 0xff; // 右移八位是因�?16位数据只有后面八位可以存�?8位的数组
    TxData[1] = ((int16_t)mypid.output) & 0xff;

		CAN2_Transmit(TxID, TxLength, TxData);
		
		printf("%f,%f,%f\n", targetValue, ema_result,mypid.output);
		
    vTaskDelayUntil(&xLastWakeTime, 1);
		 
	 }
  /* USER CODE END DR16_control */
}

/* USER CODE BEGIN Header_print_task */
/**
* @brief Function implementing the Print_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_print_task */
void print_task(void const * argument)
{
  /* USER CODE BEGIN print_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END print_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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
	HAL_UART_Transmit(&huart5,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}

int fgetc(FILE *f)               //重定向fgetc函数
{
	uint8_t ch;
	HAL_UART_Receive(&huart5,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END Application */
