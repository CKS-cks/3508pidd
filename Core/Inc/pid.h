/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用 PID 控制器（位置环 / 速度环共用）
  ******************************************************************************
  * 设计要点：
  *   1. 微分项对“测量值”求导（derivative on measurement）。
  *      普通 PID 的微分项是 Kd*(e - e_last)，当目标值突然跳变（比如每隔
  *      3 秒换一个目标位置）时微分项会产生一个巨大的冲击，容易让电机猛冲。
  *      改成对测量值求导后，目标突变不会引起微分冲击，Kd 只起“阻尼”作用。
  *
  *   2. 积分分离 + 积分限幅（anti-windup）。
  *      当输出已经饱和（比如电流已经给到最大值），而误差还在往同一个方向
  *      增大时，不再继续累加积分，避免积分越积越大、退饱和时严重超调。
  *
  *   3. 输出限幅。
  *      位置环输出的是“目标转速”，速度环输出的是“C620 电流值”，
  *      两个环都必须限幅，否则一旦参数不合适会直接满电流。
  ******************************************************************************
  */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief PID 控制器结构体
  */
typedef struct
{
    float kp;          /*!< 比例系数 */
    float ki;          /*!< 积分系数（单位：输出量 / (误差·秒)） */
    float kd;          /*!< 微分系数（作用在测量值上，起阻尼作用） */

    float out_max;     /*!< 输出上限 */
    float out_min;     /*!< 输出下限（通常为 -out_max） */

    float i_max;       /*!< 积分项单独限幅（绝对值），防止积分饱和 */

    float i_term;      /*!< 当前积分项（已经乘过 ki，单位与输出相同） */
    float last_meas;   /*!< 上一次的测量值，用于对测量值求微分 */
    uint8_t started;   /*!< 是否已经采到第一帧测量值（第一帧不算微分） */
} PID_t;

/**
  * @brief  初始化 PID 控制器
  * @param  pid      控制器对象
  * @param  kp/ki/kd 三个系数
  * @param  out_max  输出限幅（对称，下限自动取 -out_max）
  * @param  i_max    积分项限幅（绝对值）
  */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max, float i_max);

/**
  * @brief  复位 PID（清空积分和微分历史）
  * @note   电机掉线、过温保护时调用，避免重新上电瞬间积分残留导致猛冲
  */
void PID_Reset(PID_t *pid);

/**
  * @brief  运行时修改三个系数（方便在线调参）
  */
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);

/**
  * @brief  计算一次 PID 输出
  * @param  pid      控制器对象
  * @param  target   目标值
  * @param  measure  测量值
  * @param  dt       控制周期，单位：秒（本工程固定 0.001s）
  * @retval PID 输出（已限幅）
  */
float PID_Calc(PID_t *pid, float target, float measure, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
