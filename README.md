# M3508 双环 PID 位置控制（RoboMaster C 板）

用 RoboMaster **C 板**（STM32F407IGH6）通过 **CAN1** 控制 **M3508 + C620**，
双环 PID（位置环 + 速度环）让电机转到指定角度，并且**每隔 3 秒自动切换到下一个位置**。
全程不使用串口、遥控器或上位机，靠 TIM6 定时器产生 1 kHz 控制节拍。

上电后的动作序列（默认）：

```
上电保持 0°  →  3s  →  90°  →  3s  →  0°  →  3s  →  -90°  →  3s  →  0° ...循环
```

---

## 1. 控制原理

```
                        位置环(PID)                速度环(PID)
目标角度(°) ──►(-)──►  Kp·e + Ki∫e + Kd·d  ──► 目标转速 ──►(-)──►  电流指令  ──► C620 ──► M3508
                ▲          (输出限幅 ±90rpm)        ▲        (输出限幅 ±6000)              │
                │                                   │                                      │
                └──────── 输出轴角度(°) ◄──── 角度换算 ◄──── CAN 反馈 0x201 ◄──────────────┘
                                                          （转子角度/转速）
```

- **位置环**：输入角度误差（°），输出“输出轴目标转速（rpm）”。
- **速度环**：输入转速误差（rpm），输出 C620 的电流指令（-16384~16384 对应 -20A~+20A）。
- 反馈换算：C620 的机械角度是**转子**角度（0~8191/圈），除以减速比 3591/187≈19.203
  才是输出轴角度。

---

## 2. 硬件连接

| C 板 | C620 / 电机 | 说明 |
|------|-------------|------|
| CAN1 接口 CAN_H | CAN_H | 双绞线 |
| CAN1 接口 CAN_L | CAN_L | 双绞线 |
| GND | GND | 共地 |
| 24V 电源 | C620 电源输入 | 注意正负极，C620 有防反接但别赌 |
| — | 电机三相线 | 按 C620 说明书接 |

> C 板的 CAN1 使用 PD0（RX）/ PD1（TX），经板载 CAN 收发器引出到接口，不需要外接收发器。

注意事项：

1. **C620 拨码开关设为 ID = 1**（对应反馈帧 0x201，控制帧 0x200 的第 1 组）。
2. CAN 总线两端各需要 **120Ω 终端电阻**（C 板与 C620 距离近时，确认板上是否有终端电阻，
   没有就并在 CAN_H/CAN_L 之间）。
3. C620 默认 CAN 波特率 **1 Mbps**，本工程 CAN1 配置为 1 Mbps（42MHz/(7×(1+3+2))）。
4. 第一次测试请**把电机从机构上拆下来空载**，或者让轮子悬空，确认方向正确后再装回去。
5. 程序会**等收到 C620 的第一帧反馈后**才开始发电流，所以先给 C 板上电、再给 24V，
   或者反过来，都可以，不会因为总线上没有节点应答而把 CAN 打进 bus-off。

---

## 3. 编译与下载

工程是 CubeMX 生成的 Makefile 工程：

```bash
make -j8          # 生成 build/3508pidd.elf / .hex / .bin
```

用 ST-Link 下载 `build/3508pidd.hex`（STM32CubeProgrammer、Keil、CLion 均可）。
调试器还能直接看变量，见第 7 节。

---

## 4. 代码结构

| 文件 | 作用 |
|------|------|
| `Core/Src/main.c` | 初始化外设，调用 `App_Init()`，主循环调用 `App_Loop()` |
| `Core/Src/app.c` | 1 kHz 定时器（TIM6）、目标位置表、定时切换逻辑 |
| `Core/Src/motor.c` | C620 CAN 收发、反馈解析、多圈角度累计、双环调用 |
| `Core/Src/pid.c` | 通用 PID（微分先行 + 积分限幅 + 输出限幅） |
| `Core/Inc/motor.h` | **所有可调参数都在这里**（PID 增益、限幅、方向、减速比） |
| `Core/Src/stm32f4xx_it.c` | `CAN1_RX0_IRQHandler`、`TIM6_DAC_IRQHandler` |

运行流程：

```
main()
 ├─ MX_GPIO_Init() / MX_CAN1_Init()
 ├─ App_Init()
 │   ├─ Motor_Init(&g_motor, 1)      // 登记电机、初始化双环 PID
 │   ├─ Motor_CAN_Init()             // 滤波器 + 启动 CAN + 开接收中断
 │   └─ App_ControlTimerInit()       // TIM6 1 kHz 中断
 └─ while(1) App_Loop()              // 每 3 秒切换一次目标角度

TIM6 中断（1 kHz）:
 └─ App_ControlTick()
     ├─ Motor_Update()              // 位置环 → 速度环 → out_current
     └─ Motor_SendAll()             // 打包 0x200 电流帧发出去

CAN1_RX0 中断:
 └─ HAL_CAN_RxFifo0MsgPendingCallback()
     └─ Motor_OnFeedback()          // 解析 0x201~0x204，更新角度/转速/温度
```

---

## 5. 想改什么，改哪里

### 5.1 改“转到哪些位置、多久换一次”

打开 `Core/Src/app.c`：

```c
#define TARGET_INTERVAL_MS     3000U        // 每个位置停 3 秒

static const float kTargetsDeg[] =
{
    0.0f,
    90.0f,
    0.0f,
    -90.0f
};
```

- 角度单位是**输出轴角度（°）**，相对**上电时刻的位置**（上电位置自动记为 0°）。
- 想转 45°、180° 就改成 `{0, 45, 180, -45}`，数组长度随意。
- 想常驻一个位置，把数组写成 `{90.0f}` 即可。

### 5.2 改电机 ID / 用多个电机

`Core/Src/app.c` 的 `App_Init()` 里：

```c
Motor_Init(&g_motor, 1U);      // ID = 1
```

- 改成 2/3/4 时，C620 拨码也要改成对应 ID。
- `Motor_SendAll()` 已经按 ID 1~4 打包 0x200 的 8 个字节，**最多可同时控制 4 个电机**。
  要用 4 个电机，就在 `app.c` 里定义 4 个 `Motor_t` 并分别 `Motor_Init(&m[i], i+1)`，
  然后在 `App_ControlTick()` 里对每个电机调用 `Motor_Update()`，最后调用一次
  `Motor_SendAll()`。
- ID 5~8 需要改用 0x1FF 控制帧，本工程默认没做。

### 5.3 改方向 / 减速比

`Core/Inc/motor.h`：

```c
#define MOTOR_DIR                 (1.0f)   // 方向反了就改 -1.0f
#define C620_SPEED_IS_ROTOR       1        // 反馈转速单位：1=转子rpm，0=输出轴rpm
#define M3508_GEAR_RATIO          (3591.0f / 187.0f)
```

### 5.4 改 PID 和限幅

同样在 `Core/Inc/motor.h`，所有参数都有中文注释。

---

## 6. 调参步骤（建议按顺序）

> 调参前先把 `SPD_OUT_MAX_CURRENT` 调小（比如 2000），确认机械和方向没问题后再加大。

1. **确认方向**：目标给 5° 左右。
   - 电机往目标方向走 → 方向正确。
   - 电机越转越快、根本停不下来（飞车）→ 立刻断电，把 `MOTOR_DIR` 改成 `-1.0f`。
   - 电机完全不动 → 检查 CAN 线、ID、120Ω、24V、C620 是否上电（C620 上电有提示音）。

2. **先调速度环**：暂时把 `POS_KP`、`POS_KI`、`POS_KD` 设小（如 1、0、0），
   给一个 30° 左右的目标，观察 `g_motor.speed_rpm` 是否能跟上 `speed_target`：
   - 加大 `SPD_KP`，直到转速出现轻微振荡，然后回调 30%~50%；
   - 再加大 `SPD_KI` 消除稳态误差，直到出现超调再回调。

3. **再调位置环**：
   - 加大 `POS_KP` 提高响应速度（会更快到达目标）；
   - 有超调就加 `POS_KD`（阻尼）；
   - 最后加 `POS_KI` 消除到位后的静差（摩擦导致的几度误差）。

4. **限幅**：
   - `POS_OUT_MAX_RPM`：最高转速，防止机构甩得太猛；
   - `SPD_OUT_MAX_CURRENT`：最大电流，保护电机和机构；
   - `MOTOR_ANGLE_LIMIT_DEG`：软件行程限制。

默认参数在空载/轻载下比较保守，装到机构上后需要按实际负载重调。

---

## 7. 没有串口，怎么看数据？（用调试器）

`g_motor` 是全局变量，用 ST-Link 在 Keil / CLion / STM32CubeIDE 里直接添加到
Watch 或 Live Watch 即可，不需要任何 printf：

| 变量 | 含义 |
|------|------|
| `g_motor.angle_deg` | 当前输出轴角度（°） |
| `g_motor.target_deg` | 当前目标角度（°） |
| `g_motor.speed_rpm` | 输出轴转速（rpm） |
| `g_motor.out_current` | 当前电流指令（±16384） |
| `g_motor.temperature` | 电机温度（℃） |
| `g_motor.pos_pid.i_term` | 位置环积分项 |
| `g_motor.spd_pid.i_term` | 速度环积分项 |

还可以把变量拖进“示波器/曲线”窗口（Keil 的 Logic Analyzer、CLion 的 Plot），
观察角度和转速的阶跃响应。

---

## 8. 常见问题

| 现象 | 可能原因 / 处理 |
|------|----------------|
| 电机不转，C620 没反应 | CAN_H/L 接反、没共地、C620 没供电、ID 拨码不是 1 |
| 电机不转但 C620 有反馈（`angle_deg` 会变） | 电流限幅太小、PID 全 0、`Motor_IsOnline` 超时 |
| 上电就飞车 | `MOTOR_DIR` 反了，改成 `-1.0f`；或 CAN 反馈 ID 不是自己的 |
| 实际转速和设定差 19.2 倍 | `C620_SPEED_IS_ROTOR` 设置不对，试着改成 0 |
| 到目标后来回抖 | 减小 `POS_KP` / `SPD_KP`，加大 `POS_DEADBAND_DEG`，检查机械间隙 |
| 到目标差几度 | 加大 `POS_KI`；检查是否有摩擦/负载 |
| 上电角度不是 0 | 正常：本工程以**上电位置**为 0°，每次上电零点会变；需要绝对零点得加限位开关做回零 |
| 动一下就停、过一会儿又动 | 反馈超时/掉线保护在起作用，检查 CAN 线接触 |
| 运行中拔过 CAN 线，插回后没反应 | CAN 控制器可能进入了 bus-off，按一下复位键（重新上电）即可 |
| 电机发热严重 | 减小 `SPD_OUT_MAX_CURRENT`、`POS_OUT_MAX_RPM`；加长间隔时间 |

---

## 9. 安全说明

- M3508 扭矩很大，**第一次一定空载测试**，确认方向和限幅后再装机构。
- 本工程已做：输出限幅、软件角度限位、反馈超时保护（100ms 无反馈切 0）、
  过温保护（80℃ 切 0）、PID 积分限幅。
- 急停请直接断 C620 的 24V 电源，软件保护不能替代物理急停。

---

## 10. 关于 CubeMX 重新生成

- `main.c` / `stm32f4xx_it.c` 的修改都在 `USER CODE BEGIN/END` 之间，重新生成不会丢。
- **`Makefile` 会被 CubeMX 覆盖**，重新生成后需要把下面三行加回 `C_SOURCES`：

```make
Core/Src/pid.c \
Core/Src/motor.c \
Core/Src/app.c \
```

- TIM6 没有在 `.ioc` 里配置，是在 `app.c` 里手写的（`App_ControlTimerInit()`），
  重新生成不影响；如果以后在 CubeMX 里启用了 TIM6，注意不要和这里重复初始化。
- 若修改了系统时钟，TIM6 的 84 分频要重新算（APB1 定时器时钟 = 84MHz 时才是 1kHz）。
