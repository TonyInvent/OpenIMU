# 可选 FIFO 硬件时间戳

此 fork 在原采集功能上增加编译选项，**默认关闭**。关闭时保留原来的 800Hz 数据寄存器读取、16 字节 CAN 数据、量程、滤波、LED、CAN 配置和 bus-off 处理；原上位机工具继续使用原格式。

开启后使用 ICM-45686 FIFO 的硬件时间戳，改为逐 FIFO 样本采集。原上游代码及协议依据为提交 `084aacd90565bac838c9c7f650ea869d62d6522b`；TDK 官方驱动配置/勘误参考为 `tdk-invn-oss/motion.mcu.icm45686.driver` 的 `0a5bed6975b5cdeecb98f95db420f3058dd88547`。

## 构建选项

代码开关：`IMU_ENABLE_SOURCE_TIMESTAMP`，未定义时为 `0`；只有 `0` / `1` 合法。

命令行构建使用 `OPENIMU_SOURCE_TIMESTAMP`，新构建目录默认 `OFF`：

```bash
export ARM_GNU_TOOLCHAIN_ROOT=/path/to/arm-gnu-toolchain
cmake --preset debug-default
cmake --build --preset debug-default

cmake --preset debug-on
cmake --build --preset debug-on
```

也可直接配置 `-DOPENIMU_SOURCE_TIMESTAMP=ON`。CMake 缓存会保留选项，所以使用独立的 default/off/on 预设目录；不能把“第一次默认关闭”误当作复用 ON 缓存会自动关闭。

STM32CubeIDE 工程未改。要启用，在所需 Debug/Release 配置的 C Compiler → Preprocessor 中增加 `IMU_ENABLE_SOURCE_TIMESTAMP=1`，再 Clean/Rebuild，确保 `Core/Src/imu_timestamp.c` 和 `Core/Src/icm45686_fifo.c` 已由 Core 源目录纳入构建。删除定义或设为 0 则回到原模式；不要仅给单个源文件设置该宏。

更多命令见 [构建说明](../cmake/BUILDING.md)。

## 为什么使用 FIFO20

ICM FIFO 的 16 字节格式只有 8 位温度，不等于原 CAN 的 16 字节数据。为保留 16 位温度，启用模式使用 **20 字节高分辨率 FIFO packet**，同时保留原配置的 800Hz、低通和物理量单位。

FIFO20 的加速度/陀螺仪高分辨率表示固定按 ±32g / ±4000dps 缩放，驱动显式换算并饱和回原 16 位量程单位。默认 ±4g 加速度除 2、±2000dps 陀螺仪除 8；原前 16 字节布局及对应原始值单位保留。它不是把 FIFO 内部 20 字节直接发给旧解码器，也不声称开启后的传感器读数与另一个时刻的直接寄存器读数逐位相同。

硬件时间戳配置为 **1µs、普通累计模式**；关闭 delta、FSYNC 替换语义和 FIFO 压缩。使用官方示例的 snapshot 模式规避 streaming 模式的末帧读取勘误，FIFO count 按 packet 读取并执行要求的双读取。

## 开启后的 CAN 数据格式

数据 ID 继续为标准 `IMU_CAN_ID`（默认 `0x01`），FD+BRS、1M/5M 位速率不变。**开启时 payload 为 32 字节**，多字节字段全部大端：

| 字节 | 类型 | 含义 |
|---|---|---|
| 0–1 | uint16 | 原样本序号，会回绕；CAN 入队失败也消耗本次样本序号 |
| 2–13 | 6 × int16 | 原加速度 XYZ、角速度 XYZ 布局与量程单位 |
| 14–15 | int16 | 原温度原始值布局，保留 16 位 |
| 16–23 | uint64 | `source_time_us`，当前源时钟段内的硬件样本时间 |
| 24–27 | uint32 | `source_epoch`，FIFO 连续性中断时递增 |
| 28–31 | uint32 | `boot_id`，每次 MCU 启动生成的诊断 token |

保留 8 字节源时间，同时每帧携带时钟段/启动 token，是为了不依赖额外状态帧恰好成功接收来区分时间轴。32 字节是合法 CAN FD 长度。旧版 16 字节工具不应直接解释该新格式；可按 DLC 区分，关闭功能则完全回到原 16 字节格式。

例如 Python 解包原始值及时间字段：

```python
seq, ax, ay, az, gx, gy, gz, temp, source_time_us, epoch, boot_id = \
    struct.unpack(">H7hQII", payload)  # len(payload) 必须为 32
```

`boot_id` 混合 MCU UID、定时器相位和首个传感器读数，仅是无 Flash 写入的诊断 token，**不是持久唯一启动计数器、随机数安全承诺或身份认证**。碰撞可能存在；接收端仍需检查序号/时间连续性，并在收到 BOOT 公告时断开旧积分段。对必须跨掉电唯一识别每次启动的系统，应另行增加持久启动计数或显式会话握手。

## 状态帧

仅开启时，标准 ID `IMU_TIMESTAMP_STATUS_CAN_ID = IMU_CAN_ID + 0x100`（默认 `0x101`）发送 16 字节状态，在新 epoch 前尝试发送，并每秒公告一次，便于中途接入。它不计入原 `imu_sample_count` 的样本计数。

| 字节 | 含义 |
|---|---|
| 0–3 | ASCII `IMUT` |
| 4 | 此可选协议版本 1 |
| 5 | 原因：0 心跳；1 MCU 启动；2 FIFO 满；3 SPI/传输错误；4 无效 FIFO 格式；5 无效时间增量；6 传感器配置丢失/复位 |
| 6 | 时间单位 1µs |
| 7 | 数据帧长度 32 |
| 8–11 | 大端 uint32 `source_epoch` |
| 12–13 | 大端 uint16 ODR 800Hz |
| 14–15 | 保留，0 |

状态帧入 TX FIFO 成功不等于物理总线交付成功；每个数据帧自带 clock identity，因此丢失状态帧不应使接收端默默按旧 epoch 积分。两只模块仍需分配不同 CAN ID，或接到两条独立总线；此功能没有自动为多个设备分配 ID。

## 时间原点、回绕与数据不连续

`source_time_us` 是**当前 epoch 第一个有效 FIFO 样本起的传感器本地时间**，第一帧为 0，不是 UTC、STM32 uptime、CAN 发送时刻或接收机时钟。单个 IMU 的积分不需要先同步到电脑：同一个 `(bus, id, boot_id, source_epoch)` 内，相邻源时间相减即可。

原始硬件时间戳是 16 位，1µs 模式约 65.536ms 回绕。只要 FIFO 样本连续，逐帧用 uint16 模差累加到 uint64，即使一次读取很多帧也可以跨过多次回绕。64 位微秒字段本身约 584542 年才回绕；1µs 是分辨率，不是时钟绝对准确度的保证。

FIFO 满、无效 packet、SPI 部分读取失败、配置丢失或无法确认的时间增量都会显式恢复并创建新 epoch，不能把它们当作普通回绕。驱动没有假设可以从未知寄存器读取时间高位，也不猜测丢失了几圈。当前固定 800Hz 模式额外拒绝零增量和大于 10ms 的不合理单步；这只是 sanity check，不能恢复已丢的数据。

接收端应：

1. 时钟身份改变、BOOT 公告、源时间不递增时，建立新时间段，不跨段积分。
2. 同一时钟段计算 `dt_us = current_source_time_us - previous_source_time_us`。
3. 根据序号缺口标记 CAN 丢帧；端点时间差不代表缺失期间的运动数据已恢复。
4. 不使用 CAN/MQTT 到包间隔作为采样 dt。网关可以合并传输，但必须保留每帧的源时间和时钟身份。

## 调度和恢复

开启模式的 IMU 任务栈由 1024 增为 2048 字节，优先级不变；原 HAL tick、系统时钟和引脚初始化不改。新 FIFO SPI 操作使用 20ms 有限超时，配置 MREG 操作按官方至少 4µs 间隔要求保守等待 1ms，正常 FIFO 读样本不插入该等待。

每轮最多排空 32 个样本，达到上限后 self-notify 并让出一个调度间隔；启动和恢复后也主动唤醒，避免 FIFO 已过门限但没有新中断边沿时停住。FULL 是可能的数据连续性丢失事件，计数不是精确丢样数量。恢复保留原 CAN bus-off 处理；配置丢失时重初始化传感器，而不是仅清 FIFO 后假装配置仍有效。

新增调试计数：`imu_fifo_overflow_count`、`imu_fifo_invalid_count`、`imu_timestamp_error_count`、`imu_timestamp_epoch`、`imu_timestamp_boot_id`、`imu_fifo_notification_coalesced_count`。FIFO 通知合并不直接等同丢样，故不冒充原直接读取模式的 `imu_missed_count`。

## 验证范围

构建和模型测试见 [验收记录](timestamp-validation.md)。默认/关闭 BIN 与同工具链编译的上游原码逐字节一致。开启模式已做 ARM 编译、静态分析和真实 C 源码的主机 ASan/UBSan 测试；**尚未在真实 OpenIMU 上验证 FIFO 时序、温度/量程换算、总线发送及恢复**，不能把离线模型当成实物验收。

之前 C5 网关的性能测试使用原 16 字节负载；本功能开启后的 32 字节格式需另行端到端重测。
