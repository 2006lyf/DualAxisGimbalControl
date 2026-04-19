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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi088.h"
#include "gm6020.h"
#include "pid.h"
#include "gimbal_ctrl.h"
#include "protocol.h"
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
/* 全局变量声明（来自main.h） */
extern gimbal_mode_t gimbal_mode;
extern vision_target_t vision_target;
extern imu_data_t imu_data;
extern motor_feedback_t yaw_feedback;
extern motor_feedback_t pitch_feedback;

/* 任务间通信句柄 */
extern osSemaphoreId_t imu_semaphore;
extern osSemaphoreId_t control_semaphore;
extern osSemaphoreId_t usb_semaphore;
extern osMessageQueueId_t usb_queue;

/* USB接收缓冲区 */
extern uint8_t usb_rx_buffer[64];

/* 控制对象 */
extern gimbal_ctrl_t gimbal_ctrl;
/* USER CODE END Variables */
/* Definitions for Task_Gyro */
osThreadId_t Task_GyroHandle;
const osThreadAttr_t Task_Gyro_attributes = {
  .name = "Task_Gyro",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Control */
osThreadId_t Task_ControlHandle;
const osThreadAttr_t Task_Control_attributes = {
  .name = "Task_Control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_USB_Receiv */
osThreadId_t Task_USB_ReceivHandle;
const osThreadAttr_t Task_USB_Receiv_attributes = {
  .name = "Task_USB_Receiv",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Search */
osThreadId_t Task_SearchHandle;
const osThreadAttr_t Task_Search_attributes = {
  .name = "Task_Search",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_Gyro(void *argument);
void StartTask_Control(void *argument);
void StartTask_USB_Receive(void *argument);
void StartTask_Search(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of Task_Gyro */
  Task_GyroHandle = osThreadNew(StartTask_Gyro, NULL, &Task_Gyro_attributes);

  /* creation of Task_Control */
  Task_ControlHandle = osThreadNew(StartTask_Control, NULL, &Task_Control_attributes);

  /* creation of Task_USB_Receiv */
  Task_USB_ReceivHandle = osThreadNew(StartTask_USB_Receive, NULL, &Task_USB_Receiv_attributes);

  /* creation of Task_Search */
  Task_SearchHandle = osThreadNew(StartTask_Search, NULL, &Task_Search_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask_Gyro */
/**
  * @brief  Function implementing the Task_Gyro thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_Gyro */
void StartTask_Gyro(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartTask_Gyro */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    /* 1kHz 读取陀螺仪 */
    BMI088_Read(&imu_data);
    
    /* 通知控制任务数据已更新 */
    osSemaphoreRelease(imu_semaphore);
    
    /* 精确的1kHz延时 */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
  /* USER CODE END StartTask_Gyro */
}

/* USER CODE BEGIN Header_StartTask_Control */
/**
* @brief Function implementing the Task_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Control */
void StartTask_Control(void *argument)
{
  /* USER CODE BEGIN StartTask_Control */
  /* Infinite loop */
  for(;;)
  {
     /* 等待1kHz定时器信号 */
    osSemaphoreAcquire(control_semaphore, portMAX_DELAY);
    
    /* 等待陀螺仪数据（确保数据最新） */
    osSemaphoreAcquire(imu_semaphore, 0);  // 不等待，如果没有就继续
    
    /* 获取最新反馈 */
    GM6020_GetFeedback(&yaw_feedback, &pitch_feedback);
    
    /* 根据模式设置目标 */
    switch(gimbal_mode)
    {
      case GIMBAL_MODE_TRACKING:
        Gimbal_SetTarget(&gimbal_ctrl, vision_target.yaw, vision_target.pitch);
        break;
        
      case GIMBAL_MODE_SEARCH:
        /* 目标由搜索任务更新 */
        break;
        
      case GIMBAL_MODE_IDLE:
        Gimbal_SetTarget(&gimbal_ctrl, 0, 0);
        break;
        
      case GIMBAL_MODE_LOST:
        /* 保持当前位置 */
        break;
    }
    /* 执行控制算法：PID计算 + 发送电流指令 */
    Gimbal_Update(&gimbal_ctrl, &imu_data, &yaw_feedback, &pitch_feedback);
  }
  /* USER CODE END StartTask_Control */
}

/* USER CODE BEGIN Header_StartTask_USB_Receive */
/**
* @brief Function implementing the Task_USB_Receiv thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_USB_Receive */
void StartTask_USB_Receive(void *argument)
{
  /* USER CODE BEGIN StartTask_USB_Receive */
  vision_target_t target;
  static uint32_t last_target_time = 0;
  /* Infinite loop */
  for(;;)
  {
     /* 等待USB数据信号 */
    osSemaphoreAcquire(usb_semaphore, portMAX_DELAY);
    
    /* 解析USB数据 */
    if(Protocol_Parse(usb_rx_buffer, 64, &target))
    {
      /* 有效数据，存入队列 */
      osMessageQueuePut(usb_queue, &target, 0, 0);
      
      /* 更新最新目标 */
      vision_target = target;
      last_target_time = HAL_GetTick();
      
      /* 切换到跟踪模式 */
      gimbal_mode = GIMBAL_MODE_TRACKING;
    }
    
    /* 超时检测 - 500ms无数据则切搜索 */
    if(HAL_GetTick() - last_target_time > 500)  // VISION_TIMEOUT_MS
    {
      gimbal_mode = GIMBAL_MODE_SEARCH;
    }
  }
  /* USER CODE END StartTask_USB_Receive */
}

/* USER CODE BEGIN Header_StartTask_Search */
/**
* @brief Function implementing the Task_Search thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Search */
void StartTask_Search(void *argument)
{
  /* USER CODE BEGIN StartTask_Search */
  static float scan_angle = 0;
  static int8_t direction = 1;
  uint32_t tick_count = 0;
  
  /* Infinite loop */
  for(;;)
  {
     /* 只在搜索模式下运行 */
    if(gimbal_mode == GIMBAL_MODE_SEARCH)
    {
      tick_count++;
      
      /* 每10ms更新一次扫描角度 (100Hz) */
      if(tick_count % 1 == 0)
      {
        scan_angle += direction * 30.0f / 100.0f;  // SEARCH_SPEED = 30
        
        /* 边界反转 */
        if(scan_angle > 180.0f)  // SEARCH_RANGE
        {
          scan_angle = 180.0f;
          direction = -1;
        }
        if(scan_angle < -180.0f)
        {
          scan_angle = -180.0f;
          direction = 1;
        }
     /* 点头动作: 每2秒改变俯仰 */
        if((tick_count / 100) % 4 < 2)
        {
          Gimbal_SetTarget(&gimbal_ctrl, scan_angle, 15.0f);  // SEARCH_PITCH
        }
        else
        {
          Gimbal_SetTarget(&gimbal_ctrl, scan_angle, -5.0f);
        }
      }
    }
    osDelay(10);  // 100Hz
  }
  /* USER CODE END StartTask_Search */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

