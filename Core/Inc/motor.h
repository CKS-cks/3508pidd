/**
  ******************************************************************************
  * @file    motor.h
  * @brief   M3508 + C620 双环 PID 位置控制（CAN 版）
  ******************************************************************************
  * 控制结构：
  *
  *   目标角度(°) --(-)--> 位置环 PID --> 目标转速(rpm) --(-)--> 速度环 PID
  *                 ^                                         ^
  *                 |                                         |
  *             实际角度(°) <---- C620 反馈 <---- M3508 <---- 电流指令
  *
  *   - 位置环：输入角度误差，输出“输出轴目标转速”，限幅到 ±POS_OUT_MAX_RPM
  *   - 速度环：输入转速误差，输出“C620 电流指令”（-16384 ~ +16384 对应 -20A ~ +20A）
  *
  * 关键换算（重要，参数不对会导致转速/角度差 19.2 倍）：
  *   - C620 反馈的“机械角度”是【转子】角度，0~8191 对应转子一圈；
  *   - 反馈的“转速”默认按【转子 rpm】解析（可用 C620_SPEED_IS_ROTOR 切换）；
  *   - M3508 P19 减速比 3591/187 ≈ 19.203；
  *   - 所以 输出轴角度 = 转子累计计数 / 8192 * 360° / 19.203。
  ******************************************************************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include <stdint.h>

/* ==========================================================================
 *                        用户可修改的配置（重点）
 * ========================================================================== */

/* 3508 P19 减速比，精确值 3591/187 ≈ 19.203 */
#define M3508_GEAR_RATIO          (3591.0f / 187.0f)

/* C620 编码器每圈计数（机械角度 0 ~ 8191） */
#define M3508_ENCODER_CPR         8192.0f

/* 电机安装方向：若位置环一启动就“飞车”（越跑越远、停不下来），
 * 说明角度/电流的正方向定义相反，把这里改成 -1.0f 即可。 */
#define MOTOR_DIR                 (1.0f)

/* C620 反馈转速的单位：
 *   1 = 转子 rpm（默认，空载全速时读数约 9000）
 *   0 = 输出轴 rpm（空载全速时读数约 470）
 * 如果发现实际转速总是差 19.2 倍，就改这个宏。 */
#define C620_SPEED_IS_ROTOR       1

/* 软件限位：输出轴角度（相对上电位置），防止机构撞到机械限位 */
#define MOTOR_ANGLE_LIMIT_DEG     720.0f

/* 控制周期：TIM6 1 kHz 中断，固定 1 ms */
#define CONTROL_DT_S              0.001f

/* 反馈超时：超过 100ms 没收到该电调的任何 CAN 帧，就认为掉线，输出 0 */
#define MOTOR_FEEDBACK_TIMEOUT_MS 100U

/* 过温保护：C620 反馈的电机温度超过该值就停止输出 */
#define MOTOR_TEMP_LIMIT_C        80U

/* ---------------------------- 位置环参数 ---------------------------- */
/* 位置环输出 = 目标转速(rpm)，输入 = 角度误差(°)。
 * 例：POS_KP = 3，误差 30° → 目标转速 90 rpm（再被 POS_OUT_MAX_RPM 限幅）。
 * POS_KD 作用在“实际角度”上，相当于给位置环加阻尼，抑制超调。 */
#define POS_KP                    10.0f
#define POS_KI                    0.5f
#define POS_KD                    0.03f

#define POS_OUT_MAX_RPM           90.0f   /* 最大目标转速（输出轴 rpm），安全限速 */
#define POS_I_MAX_RPM             45.0f   /* 位置环积分限幅 */

/* 到位死区：误差小于 0.5° 且转速小于 5 rpm 时，目标转速直接给 0，
 * 让电机稳稳停住，避免在目标点附近来回“嗡嗡”抖动。 */
#define POS_DEADBAND_DEG          0.5f
#define POS_HOLD_SPEED_RPM        5.0f

/* ---------------------------- 速度环参数 ---------------------------- */
/* 速度环输出 = C620 电流值（±16384 对应 ±20A），输入 = 转速误差(rpm)。
 * 先调速度环 Kp，让转速能跟上且不抖；再调 Ki 消除静差。 */
#define SPD_KP                    200.0f
#define SPD_KI                    0.0f
#define SPD_KD                    0.0f

#define SPD_OUT_MAX_CURRENT       16000.0f  /* 最大电流指令，6000 ≈ 7.3A，先保守一点 */
#define SPD_I_MAX_CURRENT         2000.0f  /* 速度环积分限幅 */

/* ==========================================================================
 *                              数据结构
 * ========================================================================== */

/**
  * @brief 单个 M3508 电机的全部状态
  */
typedef struct
{
    uint8_t id;                    /*!< 电调 ID：1~4，反馈帧 0x201~0x204，控制帧 0x200 */

    /* ---- C620 原始反馈 ---- */
    uint16_t raw_angle;            /*!< 转子机械角度 0~8191 */
    int16_t raw_speed;             /*!< 原始转速（rpm，单位见 C620_SPEED_IS_ROTOR） */
    int16_t raw_current;           /*!< 实际转矩电流（-16384~16384） */
    uint8_t temperature;           /*!< 电机温度（℃） */

    /* ---- 换算后的状态 ---- */
    int32_t total_count;           /*!< 转子累计计数（多圈，掉电清零） */
    uint16_t last_raw;             /*!< 上一次原始角度，用于跨圈累计 */
    uint8_t inited;                /*!< 是否已经收到过第一帧反馈 */

    float angle_deg;               /*!< 输出轴角度（°，上电位置记为 0） */
    float speed_rpm;               /*!< 输出轴转速（rpm） */
    float target_deg;              /*!< 目标角度（°，输出轴，相对上电位置） */

    uint32_t last_rx_tick;         /*!< 最后一次收到反馈的系统 tick（ms） */

    /* ---- 双环 PID ---- */
    PID_t pos_pid;                 /*!< 位置环 */
    PID_t spd_pid;                 /*!< 速度环 */

    float out_current;             /*!< 最终输出给 C620 的电流指令 */
} Motor_t;

/* ==========================================================================
 *                              函数接口
 * ========================================================================== */

/**
  * @brief  初始化一个电机对象，并登记到 CAN 接收回调的查找表
  * @param  motor 电机对象
  * @param  id    电调 ID（1~4），必须与 C620 拨码开关一致
  */
void Motor_Init(Motor_t *motor, uint8_t id);

/**
  * @brief  设置目标角度（输出轴，°，相对上电位置）
  * @note   内部会做软件限位，超过 MOTOR_ANGLE_LIMIT_DEG 会被截断
  */
void Motor_SetTarget(Motor_t *motor, float deg);

/**
  * @brief  1 kHz 控制周期调用：位置环 + 速度环，计算 out_current
  * @note   电机掉线 / 过温时自动清零输出并复位 PID
  */
void Motor_Update(Motor_t *motor);

/**
  * @brief  电机是否在线（100ms 内收到过反馈）
  * @retval 1 = 在线，0 = 掉线
  */
uint8_t Motor_IsOnline(const Motor_t *motor);

/**
  * @brief  解析一帧 C620 反馈（在 CAN 接收中断里调用）
  * @param  motor 电机对象
  * @param  data  8 字节反馈数据
  */
void Motor_OnFeedback(Motor_t *motor, const uint8_t data[8]);

/**
  * @brief  初始化 CAN 滤波器、启动 CAN、开启接收中断
  * @note   必须在 MX_CAN1_Init() 之后调用
  */
void Motor_CAN_Init(void);

/**
  * @brief  把 4 个电机的电流打包成一帧 0x200 发出去（1 kHz 调用）
  */
void Motor_SendAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
