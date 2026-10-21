# open_imu — ICM-45686 → CAN FD 实时数据流

基于 **STM32G431CBU6** + **ICM-45686**（6 轴陀螺+加速度计，SPI 接口）的 IMU 数据采集与 CAN FD 发送工程。传感器以 **800 Hz ODR** 输出，数据通过 **CAN FD（标准帧 ID = 0x01）** 实时发送，系统基于 FreeRTOS。

设计目标：**尽可能低的端到端延迟**。

---

## 1. 硬件连接

| 外设 | 引脚 | 说明 |
|---|---|---|
| SPI1_SCK | PA5 | ICM-45686 SCK，12.5 MHz |
| SPI1_MISO | PA6 | ICM-45686 SDO |
| SPI1_MOSI | PA7 | ICM-45686 SDI |
| SPI1_NSS | PA4 | ICM-45686 CS（硬件 NSS） |
| EXTI0 / INT1 | PB0 | ICM-45686 数据就绪中断，上升沿触发 |
| FDCAN1_TX | PA12 | CAN FD 发送 |
| FDCAN1_RX | PA11 | CAN FD 接收（本工程仅发送，未用） |
| LED | PB2 | 状态 LED（灌电流：低电平点亮） |

### 1.1 状态 LED（PB2）

PB2 驱动一颗灌电流接法的状态 LED（低电平点亮）：

| 状态 | LED 行为 |
|---|---|
| 上电 / 初始化 | 常亮 |
| 正常（CAN 发送） | 每 100 帧翻转一次（约 4 Hz） |
| 运行出错（SPI/CAN） | 快闪（每 30 ms 翻转一次） |
| 致命初始化错误 | 一直快闪 |

---

## 2. 数据流架构

系统采用**「数据就绪中断驱动 + 读寄存器」**模型，而不是轮询或 FIFO 批量读取，目的是让每一帧 CAN 数据都紧跟传感器输出，且只发**最新**样本。

```
                     ┌─────────────────────────────────────────────────────┐
                     │                    ICM-45686                         │
                     │   accel(3) + gyro(3) @ 800 Hz ODR，低噪声模式         │
                     │   内部采样就绪 → INT1 引脚输出一个高电平脉冲            │
                     └───────────────┬─────────────────────────────────────┘
                                     │ INT1 (800 Hz, 1.25 ms 周期)
                                     ▼
                     ┌─────────────────────────────────────────────────────┐
                     │   PB0 = EXTI0（上升沿）                              │
                     │   EXTI0_IRQHandler → HAL_GPIO_EXTI_IRQHandler        │
                     │        → HAL_GPIO_EXTI_Callback()                    │
                     │        → IMU_DataReadyFromISR()                      │
                     │        → vTaskNotifyGiveFromISR()  （任务通知，零堆） │
                     └───────────────┬─────────────────────────────────────┘
                                     │ 直接唤醒（PendSV 上下文切换）
                                     ▼
                     ┌─────────────────────────────────────────────────────┐
                     │   imuTask（优先级 osPriorityHigh）                   │
                     │   ulTaskNotifyTake() 阻塞 → 被唤醒                   │
                     │   1) ICM45686_ReadData()   —— SPI 14 字节突发读      │
                     │   2) IMU_PackFrame()       —— 组 16 字节大端帧        │
                     │   3) HAL_FDCAN_AddMessageToTxFifoQ() —— 入发送 FIFO  │
                     └───────────────┬─────────────────────────────────────┘
                                     │
                                     ▼
                     ┌─────────────────────────────────────────────────────┐
                     │   FDCAN1 硬件（仲裁 1 Mbit/s + 数据段 5 Mbit/s，BRS）│
                     │   自动仲裁、位填充、CRC、发送                        │
                     └─────────────────────────────────────────────────────┘
```

### 2.1 关键设计选择

| 设计点 | 选择 | 对延迟的意义 |
|---|---|---|
| 采样触发 | INT1 数据就绪中断 | 精确同步到传感器 800 Hz 输出，无轮询空转 |
| ISR → 任务唤醒 | `vTaskNotifyGiveFromISR` / `ulTaskNotifyTake` | FreeRTOS 最快的唤醒原语，无队列/信号量对象与内存开销 |
| 读取方式 | 直接读数据寄存器（非 FIFO） | 始终读**最新**样本，不产生 FIFO 积压延迟 |
| 任务优先级 | `osPriorityHigh` | 被唤醒后几乎立即被调度 |
| SPI | 阻塞式 `HAL_SPI_TransmitReceive` 突发读 | 14 字节小数据量下阻塞方式最确定，无 DMA 配置开销 |
| CAN | FD + BRS，数据段 5 Mbit/s | 缩短数据段传输时间 |

### 2.2 各文件职责

```
Core/Src/icm45686.c        ICM-45686 驱动：SPI 传输、寄存器读写、软复位、
                           WHO_AM_I 校验、ODR/量程配置、14 字节突发读、端序自适应
Core/Inc/icm45686.h        寄存器地址、ODR/量程枚举、数据结构、换算系数
Core/Src/imu_task.c        FreeRTOS 任务：初始化、EXTI0 使能、组帧、FDCAN 发送、诊断计数
Core/Inc/imu_task.h        任务接口、CAN ID、帧长度
Core/Src/main.c            SPI/FDCAN 外设初始化（已修复为 8-bit / 12.5MHz / FD+BRS）
Core/Src/stm32g4xx_it.c    EXTI0_IRQHandler
```

---

## 3. CAN FD 帧格式

- **ID**：`0x01`（标准帧）
- **类型**：数据帧，FD 格式，BRS 开启（仲裁 1 Mbit/s / 数据段 5 Mbit/s）
- **长度**：16 字节，多字节字段为**大端**

```
字节      内容
[0:1]    seq        帧序号（uint16，用于检测丢帧）
[2:3]    accel X    原始值 int16
[4:5]    accel Y
[6:7]    accel Z
[8:9]    gyro X     原始值 int16
[10:11]  gyro Y
[12:13]  gyro Z
[14:15]  temperature 原始值 int16
```

**接收端换算**（当前量程 ±4g / ±2000 dps）：

```
accel_g  = raw_accel / 8192.0
gyro_dps = raw_gyro  / 16.4
temp_°C  = raw_temp  / 132.48 + 25
```

---

## 4. 延迟分析

### 4.1 延迟的组成

端到端延迟 = **传感器内部延迟** + **MCU 处理路径** + **CAN 发送路径**。

#### ① 传感器内部延迟（主导项）

ICM-45686 在低噪声模式下，内部有 ADC 转换和 UI 低通滤波（抗混叠）链路，这部分**群延迟**通常是整个链路中占比最大的部分，约 **1–3 ms 量级**（具体取决于内部 UI 低通滤波器带宽设置，见 §5.2）。这一延迟由 MEMS 传感器本身决定，与 MCU 无关，是"物理量 → 数字样本就绪"的固有代价。

#### ② MCU 处理路径（INT1 边沿 → 数据入 CAN 发送 FIFO）

| 阶段 | 典型耗时 | 说明 |
|---|---|---|
| INT1 边沿 → NVIC 进入 ISR | ~0.2–0.5 µs | Cortex-M4 中断响应 ~12 周期 + EXTI 同步（100 MHz） |
| ISR 内通知任务 | ~0.5–1 µs | HAL 回调 + `vTaskNotifyGiveFromISR` |
| PendSV 上下文切换到 imuTask | ~1–2 µs | FreeRTOS Cortex-M4F 上下文切换（含惰性 FPU 压栈） |
| SPI 14 字节突发读 | ~10–12 µs | 15 字节 × 8 bit @ 12.5 MHz ≈ 9.6 µs + HAL 开销 |
| 组帧 + 入 FDCAN 发送 FIFO | ~2–3 µs | 打包 + `HAL_FDCAN_AddMessageToTxFifoQ` |

**MCU 路径合计 ≈ 15–20 µs。**

#### ③ CAN FD 发送路径（数据入 FIFO → 上总线）

16 字节 FD 帧（1 Mbit/s 仲裁 + 5 Mbit/s 数据段，含位填充/CRC/EOF/IFS）：

| 阶段 | 典型耗时 |
|---|---|
| 仲裁段（SOF+ID+FDF+BRS+ESI+DLC，约 22 位 + 填充）@ 1 Mbit/s | ~25–30 µs |
| 数据段（128 位 + 填充 + CRC 21 位）@ 5 Mbit/s | ~35 µs |
| 结尾（ACK+EOF+IFS）@ 1 Mbit/s | ~13 µs |
| **整帧合计** | **~70–80 µs** |

> 首字节数据在**仲裁段结束后**即开始上总线，约在发送开始后 **~25–30 µs** 出现。

### 4.2 端到端总延迟

从「传感器内部样本就绪」到「接收端拿到首字节数据」：

```
总延迟 ≈ 传感器群延迟(1–3 ms) + MCU 路径(~15–20 µs) + CAN 仲裁段(~25–30 µs)
       ≈ 传感器群延迟 + ~40–50 µs
```

**结论**：MCU + CAN 带来的额外延迟只有约 40–50 µs，**真正的延迟瓶颈是传感器内部的滤波群延迟**。要压低整体延迟，应优先从传感器滤波器带宽入手（§5.2）。

### 4.3 抖动与丢帧

- 采集周期为 800 Hz = 1.25 ms，而整条 MCU+CAN 路径约 90 µs，仅占周期的 **~7%**，CPU 与总线都远未饱和，因此**抖动很小、无积压**。
- `imu_task` 用任务通知唤醒，若因更高优先级任务短暂阻塞而漏掉一次就绪脉冲，`ulTaskNotifyTake` 的返回值会 >1，`imu_missed_count` 自增；此时读到的仍是**最新**寄存器样本，**不会引入延迟**，仅丢失中间样本（接收端用 `seq` 序号可检测到跳号）。
- 若 CAN 总线拥塞导致发送 FIFO（3 级）写满，`HAL_FDCAN_AddMessageToTxFifoQ` 返回错误，`imu_can_error_count` 自增，该帧被丢弃（不会阻塞任务、不累积延迟）。

### 4.4 诊断计数器（调试器内观察）

| 变量 | 含义 |
|---|---|
| `imu_sample_count` | 已成功入 FIFO 的帧数 |
| `imu_missed_count` | 数据就绪溢出（任务未能及时响应）次数 |
| `imu_can_error_count` | CAN 发送 FIFO 满/发送失败次数 |
| `imu_spi_error_count` | SPI 读失败次数 |

---

## 5. 进一步降低延迟的选项

### 5.1 已采用（代码已实现）

- 中断驱动 + 任务通知唤醒（替代轮询/信号量）。
- 直接读最新寄存器样本（替代 FIFO 批量）。
- FDCAN FD + BRS，数据段 5 Mbit/s。

### 5.2 传感器：加宽内部 UI 低通滤波器（收益最大）

ICM-45686 的陀螺/加速度 UI 低通滤波器带宽可调，带宽越宽群延迟越低（但噪声增大）。当前驱动保持出厂默认；若要极致低延迟，可在 `icm45686.c` 的 `ICM45686_Init()` 中追加（通过 MREG 间接写）：

| 传感器 | MREG 地址 | 位 | 说明 |
|---|---|---|---|
| 陀螺 | `IPREG_SYS1_REG_172` = `0xA4AC` | `gyro_ui_lpfbw_sel[2:0]` | `0`=不滤波（最宽/最低延迟）… `6`=最窄 |
| 加速度 | `IPREG_SYS2_REG_131` = `0xA583` | `accel_ui_lpfbw_sel[2:0]` | 同上 |

写入方式与 `SREG_CTRL` 相同：先写 16 位地址到 `IREG_ADDR_15_8(0x7C)/IREG_ADDR_7_0(0x7D)`，再写数据到 `IREG_DATA(0x7E)`。设为 `0`（NO_FILTER）可获得最低群延迟。

### 5.3 SPI 时钟

当前 12.5 MHz（ICM-45686 上限 24 MHz，留了余量）。可改为 `SPI_BAUDRATEPRESCALER_4`（25 MHz，略超规格）或使用更贴近 24 MHz 的时钟源，将 14 字节突发读从 ~10 µs 降到 ~5 µs。收益相对较小（<10 µs）。

### 5.4 CAN 位速率

- 仲裁段 1 Mbit/s 是业界标准；提高到 2 Mbit/s 可把仲裁段从 ~27 µs 降到 ~14 µs，但会降低与普通 1 Mbit/s 节点的兼容性。
- 数据段 5 Mbit/s 可提升到 8 Mbit/s（STM32G4 FDCAN 上限），缩短数据段约 ~10 µs。

> 综合看，**传感器内部滤波群延迟（毫秒级）才是真正的大头**，MCU/CAN 侧的微秒级优化对总延迟影响有限；先做 §5.2 收益最高。

---

## 6. 构建与烧录

1. 用 STM32CubeIDE 打开工程，`Project → Build`（已通过 13.3 工具链全量编译验证）。
2. 烧录后，若仅本节点在总线上（无其它 CAN 节点应答），发送会因无 ACK 而失败（`imu_can_error_count` 累加），属正常总线现象；实测需接入至少一个支持 **CAN FD + BRS 且数据段 ≥5 Mbit/s** 的接收节点，并在总线两端加 **120 Ω 终端电阻**。

> 注意：若用 CubeMX 重新生成代码，会覆盖 `main.c` 里 SPI/FDCAN 的生成区初始化，需重新把 SPI 设为 8-bit / 12.5 MHz / NSS pulse 关、FDCAN 设为 FD+BRS（1M/5M）。
