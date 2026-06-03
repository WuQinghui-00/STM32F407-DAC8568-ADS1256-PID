#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float setpoint)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->measured = 0.0f;
    pid->output = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->ff = 0.0f;
    pid->out_min = 0.0f;
    pid->out_max = 5.0f;    /* DAC输出上限=5V(GAIN=2x) */
    pid->integral_limit = 20.0f;
    pid->kp_last = kp;
    pid->ki_last = ki;
    pid->kd_last = kd;
    pid->sp_last = setpoint;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

float PID_Update(PID_t *pid, float measured, float dt)
{
    float error, p_term, i_term, d_term;

    /* 检测参数变更(P/I/D)，任一变化时复位积分 */
    if (pid->kp != pid->kp_last || pid->ki != pid->ki_last ||
        pid->kd != pid->kd_last)
    {
        PID_Reset(pid);
        pid->kp_last = pid->kp;
        pid->ki_last = pid->ki;
        pid->kd_last = pid->kd;
    }
    /* 同步setpoint快照 */
    pid->sp_last = pid->setpoint;

    if (dt <= 0.0f) dt = 0.001f;

    pid->measured = measured;
    error = pid->setpoint - measured;

    /* P */
    p_term = pid->kp * error;

    /* I */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    i_term = pid->ki * pid->integral;

    /* D */
    d_term = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* 合并输出 (前馈偏置补偿DAC/ADC偏移) */
    pid->output = p_term + i_term + d_term + pid->ff;

    /* 输出限幅 */
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    return pid->output;
}