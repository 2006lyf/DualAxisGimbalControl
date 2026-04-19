/* gimbal_ctrl.h */
#ifndef GIMBAL_CTRL_H
#define GIMBAL_CTRL_H

#include "main.h"
#include "pid.h"

typedef struct {
    pid_t yaw_pos_pid;
    pid_t yaw_speed_pid;
    pid_t pitch_pos_pid;
    pid_t pitch_speed_pid;
    
    float target_yaw;
    float target_pitch;
    float current_yaw;
    float current_pitch;
    
    float yaw_output;
    float pitch_output;
    
    uint32_t last_update;
} gimbal_ctrl_t;

void Gimbal_Init(gimbal_ctrl_t *ctrl);
void Gimbal_Update(gimbal_ctrl_t *ctrl, imu_data_t *imu, motor_feedback_t *yaw_fb, motor_feedback_t *pitch_fb);
void Gimbal_SetTarget(gimbal_ctrl_t *ctrl, float yaw, float pitch);
void Gimbal_SetMode(gimbal_mode_t mode);
void Gimbal_SoftwareLimit(float *yaw, float *pitch);

#endif