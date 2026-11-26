/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 控制器实现
  ******************************************************************************
  */

#include "pid.h"

/* 把 value 限制在 [lo, hi] 之间 */
static float PID_Clamp(float value, float lo, float hi)
{
    if (value > hi)
    {
        return hi;
    }
    if (value < lo)
    {
        return lo;
    }
    return value;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max, float i_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_max = out_max;
    pid->out_min = -out_max;   /* 位置环、速度环都是对称限幅 */

    pid->i_max = i_max;

    pid->i_term = 0.0f;
    pid->last_meas = 0.0f;
    pid->started = 0;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->i_term = 0.0f;
    pid->last_meas = 0.0f;
    pid->started = 0;
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float PID_Calc(PID_t *pid, float target, float measure, float dt)
{
    float error;
    float p_term;
    float d_term;
    float output;

    if ((pid == 0) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    /* 误差：目标 - 实际 */
    error = target - measure;

    /* ---------- 比例项 ---------- */
    p_term = pid->kp * error;

    /* ---------- 微分项（对测量值求导，避免目标突变时的微分冲击） ---------- */
    d_term = 0.0f;
    if (pid->started != 0U)
    {
        d_term = -pid->kd * (measure - pid->last_meas) / dt;
    }
    pid->last_meas = measure;
    pid->started = 1;

    /* ---------- 积分项 ---------- */
    /* 先按“还没积分”的输出判断是否已经饱和：
     * 如果输出已经顶到上限且误差为正（还想往上顶），或者已经顶到下限且
     * 误差为负（还想往下顶），就冻结积分，防止积分饱和。 */
    output = p_term + pid->i_term + d_term;

    if (!(((output >= pid->out_max) && (error > 0.0f)) ||
          ((output <= pid->out_min) && (error < 0.0f))))
    {
        pid->i_term += pid->ki * error * dt;

        /* 积分项单独限幅，双保险 */
        pid->i_term = PID_Clamp(pid->i_term, -pid->i_max, pid->i_max);
    }

    /* ---------- 合成输出并限幅 ---------- */
    output = p_term + pid->i_term + d_term;

    return PID_Clamp(output, pid->out_min, pid->out_max);
}
