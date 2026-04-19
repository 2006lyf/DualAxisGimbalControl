/* pid.c */
#include "pid.h"

void PID_Init(pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    
    pid->deadband = 0.0f;
    pid->feedforward = 0.0f;
    
    PID_Reset(pid);
}

void PID_Reset(pid_t *pid)
{
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

void PID_SetDeadband(pid_t *pid, float deadband)
{
    pid->deadband = deadband;
}

void PID_SetFeedforward(pid_t *pid, float ff)
{
    pid->feedforward = ff;
}

float PID_Calc(pid_t *pid, float ref, float fdb)
{
    pid->error = ref - fdb;
    
    // 死区处理
    if(ABS(pid->error) < pid->deadband) {
        pid->error = 0;
    }
    
    // 积分项
    pid->integral += pid->error;
    
    // 积分限幅
    if(pid->integral > pid->max_iout) pid->integral = pid->max_iout;
    if(pid->integral < -pid->max_iout) pid->integral = -pid->max_iout;
    
    // 微分项 (带低通滤波的近似)
    float derivative = pid->error - pid->last_error;
    
    // 计算输出
    pid->output = pid->kp * pid->error +
                  pid->ki * pid->integral +
                  pid->kd * derivative +
                  pid->feedforward;
    
    // 输出限幅
    if(pid->output > pid->max_out) pid->output = pid->max_out;
    if(pid->output < -pid->max_out) pid->output = -pid->max_out;
    
    pid->last_error = pid->error;
    
    return pid->output;
}