#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float setpoint;     // 目标值(伏)
    float measured;     // 测量值(伏)
    float output;       // 输出值(伏)
    float integral;     // 积分累计
    float prev_error;   // 上次误差(用于微分)
    float out_min;      // 输出下限
    float out_max;      // 输出上限
    float integral_limit; // 积分限幅(抗饱和)
    float ff;             // 前馈偏置(V)，补偿DAC/ADC偏移
    float kp_last;        // 上次参数快照(检测变更用)
    float ki_last;
    float kd_last;
    float sp_last;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float setpoint);
void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float measured, float dt);

#endif