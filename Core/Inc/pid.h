/* pid.h */
#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_out;
    float max_iout;
    
    float error;
    float last_error;
    float integral;
    float output;
    
    float deadband;      // 死区
    float feedforward;   // 前馈
} pid_t;

void PID_Init(pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout);
void PID_Reset(pid_t *pid);
float PID_Calc(pid_t *pid, float ref, float fdb);
void PID_SetDeadband(pid_t *pid, float deadband);
void PID_SetFeedforward(pid_t *pid, float ff);

#endif