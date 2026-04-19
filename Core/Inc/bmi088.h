/* bmi088.h */
#ifndef BMI088_H
#define BMI088_H

#include "main.h"

/* BMI088 寄存器地址 */
#define BMI088_ACC_CHIP_ID     0x00  // 加速度计ID寄存器
#define BMI088_GYRO_CHIP_ID    0x00  // 陀螺仪ID寄存器

#define BMI088_ACC_DATA        0x12  // 加速度数据起始
#define BMI088_GYRO_DATA       0x02  // 陀螺仪数据起始

#define BMI088_ACC_PWR_CTRL    0x7D  // 电源控制
#define BMI088_ACC_PWR_CONF    0x7C  // 电源配置
#define BMI088_ACC_CONF        0x40  // 加速度计配置
#define BMI088_ACC_RANGE       0x41  // 量程配置

#define BMI088_GYRO_RANGE      0x0F  // 陀螺仪量程
#define BMI088_GYRO_BANDWIDTH  0x10  // 带宽
#define BMI088_GYRO_LPM1       0x11  // 低功耗模式

/* 片选引脚定义 (根据你的主控手册) */
#define ACCEL_CS_LOW()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)
#define ACCEL_CS_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)
#define GYRO_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

uint8_t BMI088_Init(void);
void BMI088_Read(imu_data_t *data);
void BMI088_ReadGyro(float gyro[3]);
void BMI088_ReadAccel(float accel[3]);
float BMI088_GetTemp(void);

#endif