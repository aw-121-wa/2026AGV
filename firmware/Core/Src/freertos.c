/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hwt101.h"
#include "zdt_x42s.h"
#include "chassis.h"
#include "chassis_cmd.h"
#include "agv_run.h"
#include "competition.h"
#include "arm_app.h"
#include "tune_log.h"
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
/* USER CODE BEGIN Variables */
#define IMU_RX_FLAG      0x0001U
#define IMU_WAIT_TIMEOUT 10U /* ms，兜底：串口停流时也定期检查 DMA 重启 */
volatile const char *stack_overflow_task; /* 调试器查看溢出的任务名 */

/* USER CODE END Variables */
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void IMU_RxNotify(void);

/* USER CODE END FunctionPrototypes */

void StartIMUTask(void *argument);
void StartChassisTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
/* 栈溢出：尽力让所有电机（四个轮子、机械臂）立即停止（FDCAN 由硬件发送，不依赖 RTOS），然后停机等调试。 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  taskDISABLE_INTERRUPTS();
  stack_overflow_task = pcTaskName;
  ZDT_EmergencyStop();
  for (;;)
  {
  }
}
/* USER CODE END 4 */

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
  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  (void)argument;
  HWT101_SetRxHook(IMU_RxNotify);
  /* Infinite loop */
  for(;;)
  {
    /* 串口中断收到数据后才唤醒，不再 1 ms 空转轮询。 */
    (void)osThreadFlagsWait(IMU_RX_FLAG, osFlagsWaitAny, IMU_WAIT_TIMEOUT);
    HWT101_ProcessData(&huart5, &imu);
  }
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  (void)argument;

  const uint32_t period = (osKernelGetTickFreq() * CHASSIS_PERIOD_US) / 1000000U;

  uint32_t next_tick = osKernelGetTickCount();

  for (;;) {
    Chassis_Update();

    TuneLog_Tick();

    ChassisCmd_Poll();

    Agv_Tick();

#if AGV_MODE == AGV_MODE_COMPETITION
    Comp_Tick();
#elif AGV_MODE == AGV_MODE_ARM
    ArmApp_Tick();
#endif

    Chassis_CycleEnd();

    next_tick += period;

    if ((int32_t)(next_tick - osKernelGetTickCount()) <= 0)
      next_tick = osKernelGetTickCount() + period;
    
    (void)osDelayUntil(next_tick);
  }
  /* USER CODE END StartChassisTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void IMU_RxNotify(void)
{
  (void)osThreadFlagsSet(IMUTaskHandle, IMU_RX_FLAG);
}

/* USER CODE END Application */

