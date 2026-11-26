/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层：1 kHz 控制节拍 + 定时切换目标位置
  ******************************************************************************
  */

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor.h"

/**
  * @brief 全局电机对象
  * @note  定义成全局变量，方便用调试器（ST-Link/Keil/CLion）直接 watch：
  *        g_motor.angle_deg、g_motor.target_deg、g_motor.speed_rpm、
  *        g_motor.out_current、g_motor.temperature
  */
extern Motor_t g_motor;

/**
  * @brief  应用初始化：电机对象、CAN、1 kHz 定时器
  * @note   在 main() 的 MX_CAN1_Init() 之后调用
  */
void App_Init(void);

/**
  * @brief  主循环任务：每隔 TARGET_INTERVAL_MS 切换一次目标位置
  * @note   放在 while(1) 里反复调用，内部用 HAL_GetTick() 计时，不阻塞
  */
void App_Loop(void);

/**
  * @brief  1 kHz 控制中断里调用：双环 PID 计算 + CAN 发送
  * @note   由 TIM6 更新中断调用，不要在里面做耗时操作
  */
void App_ControlTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
