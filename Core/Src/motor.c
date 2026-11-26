/**
  ******************************************************************************
  * @file    motor.c
  * @brief   M3508 + C620 双环 PID 位置控制实现
  ******************************************************************************
  * CAN 协议（大疆 C620 电调）：
  *
  *   控制帧（STM32 -> 电调）：标准帧 ID = 0x200（ID 1~4 用 0x200，
  *   ID 5~8 用 0x1FF）。8 字节数据 = 4 个 int16，大端序，依次对应
  *   ID1、ID2、ID3、ID4 的电流指令，范围 -16384 ~ +16384（对应 -20A~+20A）。
  *
  *   反馈帧（电调 -> STM32）：标准帧 ID = 0x201 ~ 0x204 对应 ID1~ID4，
  *   8 字节数据：
  *       data[0..1]：转子机械角度 0~8191（大端）
  *       data[2..3]：转速 rpm（int16，大端）
  *       data[4..5]：实际转矩电流（int16，大端）
  *       data[6]   ：电机温度（℃）
  *       data[7]   ：保留
  ******************************************************************************
  */

#include "motor.h"
#include "can.h"
#include "main.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* 0x200 一帧最多控制 4 个电调 */
#define MOTOR_COUNT 4

/* 电机对象查找表：下标 = ID - 1，CAN 接收中断里靠它找到对应的 Motor_t */
static Motor_t *s_motor_map[MOTOR_COUNT];

/* 已登记的电机数量：为 0 时不必往总线上发空帧 */
static uint8_t s_motor_count = 0U;

/* 角度换算系数：转子计数 -> 输出轴角度（°） */
#define COUNT_TO_OUTPUT_DEG  (360.0f / M3508_ENCODER_CPR / M3508_GEAR_RATIO)

/* -------------------------------------------------------------------------- */
/*                                 初始化                                     */
/* -------------------------------------------------------------------------- */

void Motor_Init(Motor_t *motor, uint8_t id)
{
    if ((motor == NULL) || (id < 1U) || (id > MOTOR_COUNT))
    {
        return;
    }

    /* 先把整个结构体清零，避免上电随机值 */
    memset(motor, 0, sizeof(Motor_t));

    motor->id = id;

    /* 位置环：输入角度误差(°)，输出输出轴目标转速(rpm) */
    PID_Init(&motor->pos_pid, POS_KP, POS_KI, POS_KD,
             POS_OUT_MAX_RPM, POS_I_MAX_RPM);

    /* 速度环：输入转速误差(rpm)，输出 C620 电流指令 */
    PID_Init(&motor->spd_pid, SPD_KP, SPD_KI, SPD_KD,
             SPD_OUT_MAX_CURRENT, SPD_I_MAX_CURRENT);

    /* 上电时把目标设为 0°（即上电位置），先原地保持，防止一上电就乱转 */
    motor->target_deg = 0.0f;

    /* 登记到查找表 */
    if (s_motor_map[id - 1U] == NULL)
    {
        s_motor_count++;
    }
    s_motor_map[id - 1U] = motor;
}

void Motor_SetTarget(Motor_t *motor, float deg)
{
    if (motor == NULL)
    {
        return;
    }

    /* 软件限位，防止目标角度超出机构行程 */
    if (deg > MOTOR_ANGLE_LIMIT_DEG)
    {
        deg = MOTOR_ANGLE_LIMIT_DEG;
    }
    else if (deg < -MOTOR_ANGLE_LIMIT_DEG)
    {
        deg = -MOTOR_ANGLE_LIMIT_DEG;
    }

    motor->target_deg = deg;
}

/* -------------------------------------------------------------------------- */
/*                               反馈解析                                     */
/* -------------------------------------------------------------------------- */

void Motor_OnFeedback(Motor_t *motor, const uint8_t data[8])
{
    uint16_t raw_angle;
    int16_t delta;

    if ((motor == NULL) || (data == NULL))
    {
        return;
    }

    /* 大端序拼出 16 位原始角度（0~8191） */
    raw_angle = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);

    if (motor->inited == 0U)
    {
        /* 第一帧：把当前位置记为 0°，之后所有目标角度都相对于它 */
        motor->inited = 1U;
        motor->last_raw = raw_angle;
        motor->total_count = 0;
    }
    else
    {
        /* 计算与上一帧的差值。角度会在 0/8191 之间回绕，
         * 先按 int16 求差，再把“跨圈”的差值折算到 ±4096 以内。
         * 1kHz 下电机 1ms 走的计数远小于半圈，所以这种折算不会出错。 */
        delta = (int16_t)(raw_angle - motor->last_raw);

        if (delta > 4096)
        {
            delta = (int16_t)(delta - 8192);
        }
        else if (delta < -4096)
        {
            delta = (int16_t)(delta + 8192);
        }

        motor->total_count += delta;
        motor->last_raw = raw_angle;
    }

    /* 原始转速、电流、温度 */
    motor->raw_speed = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    motor->raw_current = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
    motor->temperature = data[6];

    /* 转子累计计数 -> 输出轴角度（多圈，单位 °） */
    motor->angle_deg = MOTOR_DIR * ((float)motor->total_count) * COUNT_TO_OUTPUT_DEG;

    /* 反馈转速 -> 输出轴转速（rpm） */
#if C620_SPEED_IS_ROTOR
    motor->speed_rpm = MOTOR_DIR * ((float)motor->raw_speed) / M3508_GEAR_RATIO;
#else
    motor->speed_rpm = MOTOR_DIR * ((float)motor->raw_speed);
#endif

    /* 记录收帧时间，用于掉线检测 */
    motor->last_rx_tick = HAL_GetTick();
}

uint8_t Motor_IsOnline(const Motor_t *motor)
{
    if ((motor == NULL) || (motor->inited == 0U))
    {
        return 0U;
    }

    /* 无符号减法，tick 溢出也能正确判断 */
    if ((HAL_GetTick() - motor->last_rx_tick) >= MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}

/* -------------------------------------------------------------------------- */
/*                              双环控制                                      */
/* -------------------------------------------------------------------------- */

void Motor_Update(Motor_t *motor)
{
    float pos_error;
    float speed_target;

    if (motor == NULL)
    {
        return;
    }

    /* 掉线或过温：立刻切断输出，并清空两个环的积分，防止恢复瞬间猛冲 */
    if ((Motor_IsOnline(motor) == 0U) || (motor->temperature >= MOTOR_TEMP_LIMIT_C))
    {
        motor->out_current = 0.0f;
        PID_Reset(&motor->pos_pid);
        PID_Reset(&motor->spd_pid);
        return;
    }

    pos_error = motor->target_deg - motor->angle_deg;

    /* ---------- 外环：位置环，输出目标转速 ---------- */
    speed_target = PID_Calc(&motor->pos_pid, motor->target_deg, motor->angle_deg, CONTROL_DT_S);

    /* 到位死区：误差和转速都很小的时候，把目标转速压成 0，
     * 由速度环主动“刹住”，避免在目标点附近来回抖动。 */
    if ((fabsf(pos_error) < POS_DEADBAND_DEG) &&
        (fabsf(motor->speed_rpm) < POS_HOLD_SPEED_RPM))
    {
        speed_target = 0.0f;
    }

    /* ---------- 内环：速度环，输出电流指令 ---------- */
    motor->out_current = PID_Calc(&motor->spd_pid, speed_target, motor->speed_rpm, CONTROL_DT_S);
}

/* -------------------------------------------------------------------------- */
/*                              CAN 收发                                      */
/* -------------------------------------------------------------------------- */

void Motor_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    /* 只接收 0x200 ~ 0x20F 这一段 ID（C620 的反馈帧 0x201~0x204 都在里面），
     * 其余总线数据直接过滤掉，减少中断负担。 */
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint32_t)(0x200U << 5);      /* 标准 ID 在寄存器高 11 位 */
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = (uint32_t)(0x7F0U << 5);  /* 掩码：只比较高 7 位 */
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;                  /* 只给 CAN2 用，CAN1 忽略 */

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 开启“FIFO0 收到新消息”中断 */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    /* HAL 的 ActivateNotification 只开外设中断，NVIC 需要自己开。
     * 接收中断优先级 1，高于 1kHz 控制中断（2），保证反馈不丢帧。 */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void Motor_SendAll(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0};
    uint32_t tx_mailbox;
    uint8_t i;

    if (s_motor_count == 0U)
    {
        return;
    }

    /* 把每个电机的电流按 ID 顺序填进 8 字节（大端序） */
    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        Motor_t *motor = s_motor_map[i];
        int16_t current = 0;

        if (motor != NULL)
        {
            float out = motor->out_current;

            /* 最后一道限幅，保证不超出 C620 允许范围 */
            if (out > 16384.0f)
            {
                out = 16384.0f;
            }
            else if (out < -16384.0f)
            {
                out = -16384.0f;
            }

            current = (int16_t)out;
        }

        data[2U * i] = (uint8_t)((uint16_t)current >> 8);
        data[2U * i + 1U] = (uint8_t)((uint16_t)current & 0x00FFU);
    }

    tx_header.StdId = 0x200;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    (void)HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

/**
  * @brief  CAN FIFO0 接收中断回调（HAL 在 CAN1_RX0_IRQHandler 里调用）
  * @note   一次中断可能积压多帧，这里循环把 FIFO 读空
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
    uint32_t pending;

    if (hcan->Instance != CAN1)
    {
        return;
    }

    pending = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);

    while (pending > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
        {
            break;
        }

        /* 只处理 0x201~0x204 的标准帧 */
        if ((rx_header.IDE == CAN_ID_STD) &&
            (rx_header.StdId >= 0x201U) && (rx_header.StdId <= 0x204U))
        {
            uint8_t index = (uint8_t)(rx_header.StdId - 0x201U);

            if (s_motor_map[index] != NULL)
            {
                Motor_OnFeedback(s_motor_map[index], data);
            }
        }

        pending = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
    }
}
