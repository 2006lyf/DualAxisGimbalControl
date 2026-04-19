/* gimbal_ctrl.c */
#include "gimbal_ctrl.h"
#include "gm6020.h"

void Gimbal_Init(gimbal_ctrl_t *ctrl)
{
    /* PID参数初始化 (需要调试优化) */
    PID_Init(&ctrl->yaw_pos_pid, 5.0f, 0.0f, 0.1f, 300.0f, 100.0f);
    PID_Init(&ctrl->yaw_speed_pid, 15.0f, 0.1f, 0.0f, 8000.0f, 2000.0f);
    PID_Init(&ctrl->pitch_pos_pid, 5.0f, 0.0f, 0.1f, 300.0f, 100.0f);
    PID_Init(&ctrl->pitch_speed_pid, 15.0f, 0.1f, 0.0f, 8000.0f, 2000.0f);
    
    /* 设置死区 */
    PID_SetDeadband(&ctrl->yaw_pos_pid, 0.5f);
    PID_SetDeadband(&ctrl->pitch_pos_pid, 0.5f);
    
    ctrl->target_yaw = 0;
    ctrl->target_pitch = 0;
    ctrl->current_yaw = 0;
    ctrl->current_pitch = 0;
    ctrl->last_update = 0;
}

void Gimbal_SoftwareLimit(float *yaw, float *pitch)
{
    if(*yaw > YAW_MAX_ANGLE) *yaw = YAW_MAX_ANGLE;
    if(*yaw < YAW_MIN_ANGLE) *yaw = YAW_MIN_ANGLE;
    if(*pitch > PITCH_MAX_ANGLE) *pitch = PITCH_MAX_ANGLE;
    if(*pitch < PITCH_MIN_ANGLE) *pitch = PITCH_MIN_ANGLE;
}

void Gimbal_Update(gimbal_ctrl_t *ctrl, imu_data_t *imu, motor_feedback_t *yaw_fb, motor_feedback_t *pitch_fb)
{
    /* 获取当前角度 */
    ctrl->current_yaw = GM6020_AngleToDegree(yaw_fb->angle);
    ctrl->current_pitch = GM6020_AngleToDegree(pitch_fb->angle);
    
    /* 软件限位目标 */
    float target_yaw = ctrl->target_yaw;
    float target_pitch = ctrl->target_pitch;
    Gimbal_SoftwareLimit(&target_yaw, &target_pitch);
    
    /* 串级PID控制 */
    // 位置环 -> 速度目标
    float yaw_speed_target = PID_Calc(&ctrl->yaw_pos_pid, target_yaw, ctrl->current_yaw);
    float pitch_speed_target = PID_Calc(&ctrl->pitch_pos_pid, target_pitch, ctrl->current_pitch);
    
    // 速度环限幅
    if(yaw_speed_target > 300.0f) yaw_speed_target = 300.0f;
    if(yaw_speed_target < -300.0f) yaw_speed_target = -300.0f;
    if(pitch_speed_target > 300.0f) pitch_speed_target = 300.0f;
    if(pitch_speed_target < -300.0f) pitch_speed_target = -300.0f;
    
    /* 速度环 -> 电流输出 */
    ctrl->yaw_output = PID_Calc(&ctrl->yaw_speed_pid, yaw_speed_target, imu->gyro[0]);
    ctrl->pitch_output = PID_Calc(&ctrl->pitch_speed_pid, pitch_speed_target, imu->gyro[1]);
    
    /* 发送到电机 */
    GM6020_SetCurrent((int16_t)ctrl->yaw_output, (int16_t)ctrl->pitch_output);
    
    ctrl->last_update = HAL_GetTick();
}

void Gimbal_SetTarget(gimbal_ctrl_t *ctrl, float yaw, float pitch)
{
    ctrl->target_yaw = yaw;
    ctrl->target_pitch = pitch;
}