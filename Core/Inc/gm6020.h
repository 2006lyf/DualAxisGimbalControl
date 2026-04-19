/* gm6020.h */
#ifndef GM6020_H
#define GM6020_H

#include "main.h"

/* CAN ID 定义 (根据实际修改) */
#define CAN_CMD_ID          0x1FF  // 控制指令ID
#define YAW_FB_ID           0x205  // Yaw电机反馈ID
#define PITCH_FB_ID         0x206  // Pitch电机反馈ID

/* 电流限幅 */
#define CURRENT_MAX         16384
#define CURRENT_MIN         -16384

void GM6020_Init(void);
void GM6020_SetCurrent(int16_t yaw_current, int16_t pitch_current);
void GM6020_GetFeedback(motor_feedback_t *yaw, motor_feedback_t *pitch);
float GM6020_AngleToDegree(int16_t angle);
int16_t GM6020_DegreeToAngle(float degree);

#endif