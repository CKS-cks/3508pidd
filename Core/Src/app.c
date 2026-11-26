/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层：1 kHz 控制节拍 + 定时切换目标位置
  ******************************************************************************
  * 本文件实现两件事：
  *
  *   1. 用 TIM6 产生 1 kHz 中断，在中断里做双环 PID 计算并通过 CAN 发电流；
  *      （不用串口、不用外部信号，纯定时器节拍）
  *
  *   2. 在主循环里每隔 TARGET_INTERVAL_MS 毫秒，把目标角度切换到
  *      kTargetsDeg[] 里的下一个位置，循环往复。
  *
  * 想改“转到哪些位置 / 多久换一个”，只需要改下面两个地方：
  *   - kTargetsDeg[]      目标角度表（单位：°，相对上电位置）
  *   - TARGET_INTERVAL_MS 每个位置停留的时间（单位：ms）
  ******************************************************************************
  */

#include "app.h"
#include "main.h"

/* ==========================================================================
 *                              用户可修改的配置
 * ========================================================================== */

/* 每个目标位置停留的时间：3000 ms = 3 秒换一个位置 */
#define TARGET_INTERVAL_MS     3000U

/* 目标位置表（输出轴角度，单位 °，相对上电位置；正负对应两个方向）
 * 这里演示：0° → 90° → 0° → -90° → 0° ... 循环。
 * 想转别的角度直接改这个数组，比如 { 0, 45, 180, 270 }。 */
static const float kTargetsDeg[] =
{
    0.0f,
    90.0f,
    0.0f,
    -90.0f
};

#define TARGET_COUNT  (sizeof(kTargetsDeg) / sizeof(kTargetsDeg[0]))

/* ==========================================================================
 *                              私有变量
 * ========================================================================== */

/* 全局电机对象（ID = 1，对应 C620 拨码 1，反馈帧 0x201） */
Motor_t g_motor;

/* 上一次切换目标的时刻（HAL_GetTick 计数） */
static uint32_t s_last_target_tick = 0U;

/* 当前目标在 kTargetsDeg[] 里的下标 */
static uint32_t s_target_index = 0U;

/* 是否已经开始按节拍运动（要等第一帧反馈到了才开始，避免上电乱冲） */
static uint8_t s_motion_started = 0U;

/* ==========================================================================
 *                              私有函数
 * ========================================================================== */

/**
  * @brief  初始化 TIM6，产生 1 kHz 更新中断
  * @note   APB1 定时器时钟 = 84 MHz（系统 168 MHz，APB1 四分频 42 MHz，
  *         定时器时钟再乘 2）。84 分频得到 1 MHz，再计 1000 个数就是 1 ms。
  */
static void App_ControlTimerInit(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM6->PSC = 84U - 1U;        /* 84 MHz / 84 = 1 MHz */
    TIM6->ARR = 1000U - 1U;      /* 1 MHz / 1000 = 1 kHz */

    /* 产生一次更新事件，立刻把 PSC/ARR 装载进影子寄存器 */
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0U;               /* 清掉刚才 UG 产生的更新标志 */

    TIM6->DIER = TIM_DIER_UIE;   /* 允许更新中断 */
    TIM6->CR1 = TIM_CR1_CEN;     /* 启动定时器 */

    /* 优先级 2：低于 CAN 接收（1），保证反馈解析不被控制任务打断 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ==========================================================================
 *                              对外接口
 * ========================================================================== */

void App_Init(void)
{
    /* 1. 初始化电机对象（ID=1） */
    Motor_Init(&g_motor, 1U);

    /* 2. 配置 CAN 滤波器、启动 CAN、打开接收中断 */
    Motor_CAN_Init();

    /* 3. 启动 1 kHz 控制定时器 */
    App_ControlTimerInit();

    /* 4. 状态复位 */
    s_target_index = 0U;
    s_motion_started = 0U;
    s_last_target_tick = HAL_GetTick();
}

void App_Loop(void)
{
    /* 没收到反馈之前什么都不做（此时 Motor_Update 也会输出 0） */
    if (Motor_IsOnline(&g_motor) == 0U)
    {
        return;
    }

    /* 第一次收到反馈：从表中第 0 个位置开始，并重新计时 */
    if (s_motion_started == 0U)
    {
        s_motion_started = 1U;
        s_last_target_tick = HAL_GetTick();
        Motor_SetTarget(&g_motor, kTargetsDeg[0]);
        return;
    }

    /* 到时间了：切换到下一个目标位置 */
    if ((HAL_GetTick() - s_last_target_tick) >= TARGET_INTERVAL_MS)
    {
        s_last_target_tick = HAL_GetTick();
        s_target_index = (s_target_index + 1U) % TARGET_COUNT;
        Motor_SetTarget(&g_motor, kTargetsDeg[s_target_index]);
    }
}

void App_ControlTick(void)
{
    /* 双环 PID：位置环 -> 速度环 -> out_current */
    Motor_Update(&g_motor);

    /* 收到过反馈才开始发控制帧：
     * 如果 C620 还没上电，总线上没有任何节点回应 ACK，1 kHz 持续发送会让
     * CAN 控制器很快进入 bus-off；等第一帧反馈到了再发，就不会有这个问题，
     * 也就不用关心“先给板子上电还是先给 C620 上电”。 */
    if (g_motor.inited != 0U)
    {
        /* 把 4 路电流打包成 0x200 发出去（只配了 1 个电机，其余补 0） */
        Motor_SendAll();
    }
}
