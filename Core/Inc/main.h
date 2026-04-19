/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"  // FreeRTOS
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* 云台工作模式 */
typedef enum {
    GIMBAL_MODE_IDLE = 0,      // 待机
    GIMBAL_MODE_SEARCH,        // 哨兵模式-自动搜索
    GIMBAL_MODE_TRACKING,      // 自瞄模式-跟踪目标
    GIMBAL_MODE_LOST,          // 目标丢失-缓冲模式
} gimbal_mode_t;

/* 陀螺仪数据结构 */
typedef struct {
    float gyro[3];      // 陀螺仪数据 (deg/s)  [0]:yaw, [1]:pitch, [2]:roll
    float accel[3];     // 加速度计数据 (g)
    float temp;         // 温度
} imu_data_t;

/* 视觉目标数据结构 */
typedef struct {
    float yaw;          // 目标yaw角度 (度)
    float pitch;        // 目标pitch角度 (度)
    float distance;     // 距离 (m)
    uint8_t confidence; // 可信度 0-100
    uint32_t timestamp; // 时间戳
} vision_target_t;

/* 电机反馈数据结构 */
typedef struct {
    int16_t angle;      // 角度 (0-8191)
    int16_t speed;      // 转速 (rpm)
    int16_t current;    // 电流 (mA)
    uint8_t temp;       // 温度 (℃)
    uint8_t reserved;
} motor_feedback_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* 云台机械限位 (根据实际调整) */
#define YAW_MAX_ANGLE     180.0f
#define YAW_MIN_ANGLE    -180.0f
#define PITCH_MAX_ANGLE    30.0f
#define PITCH_MIN_ANGLE   -30.0f

/* 电机参数 */
#define MOTOR_ANGLE_MAX    8191.0f  // 编码器最大值
#define MOTOR_ANGLE_MIN    0.0f

/* 搜索模式参数 */
#define SEARCH_SPEED        30.0f   // 30度/秒
#define SEARCH_RANGE        180.0f  // ±180度
#define SEARCH_PITCH        15.0f   // 俯仰15度

/* USB超时时间 (ms) */
#define VISION_TIMEOUT_MS   500

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define LIMIT(x, max, min)  (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#define ABS(x)              (((x) > 0) ? (x) : -(x))
#define DEG2RAD(x)          ((x) * 0.01745329252f)
#define RAD2DEG(x)          ((x) * 57.295779513f)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/* 全局变量声明 */
extern gimbal_mode_t gimbal_mode;
extern vision_target_t vision_target;
extern imu_data_t imu_data;
extern motor_feedback_t yaw_feedback;
extern motor_feedback_t pitch_feedback;

/* 任务间通信句柄声明 */
extern osSemaphoreId_t imu_semaphore;
extern osSemaphoreId_t control_semaphore;
extern osSemaphoreId_t usb_semaphore;
extern osMessageQueueId_t usb_queue;

/* USB接收缓冲区 */
extern uint8_t usb_rx_buffer[64];

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
