# 可选时间戳功能验证

日期：2026-11-05。工具链：官方 Arm GNU Toolchain 14.3.Rel1，GCC 14.3.1，Cortex-M4 / fpv4-sp-d16 / hard-float。STM32CubeIDE 工程配置未修改，命令行构建配置依据原 `.cproject`。

## 结果

| 构建 | 时间戳选项 | 完整 BIN | 静态 RAM（含原 RTOS heap 预留） | 结果 |
|---|---|---:|---:|---|
| Debug 默认 | OFF | 28640B | 14120B | 编译、链接通过，无警告 |
| Debug 显式关闭 | OFF | 28640B | 14120B | 编译、链接通过，无警告 |
| Debug 开启 | ON | 33116B | 14168B | 编译、链接通过，无警告 |
| Release 默认 | OFF | 17252B | 14112B | 编译、链接通过，无警告 |
| Release 开启 | ON | 20012B | 14160B | 编译、链接通过，无警告 |

所有构建输出 ELF、BIN、HEX、MAP 和 stack-usage 报告，位于 `build/<preset>/`。新任务栈从同一 RTOS heap 分配，因此 ON 栈由 1024 增至 2048 字节的变化不等于静态 RAM 表只增加的 48 字节；运行期 heap/stack 仍需上板测量。

## 默认关闭的兼容性

将上游提交 `084aacd90565bac838c9c7f650ea869d62d6522b` 的原源码放入临时目录，仅使用同一套命令行构建定义和相同工具链进行对比：

**上游原码、fork 默认关闭、fork 显式关闭的完整 Flash BIN 逐字节完全相同**，均为 28640 字节，SHA256：

```text
0c933015b01277b09caf9a780f2ddc033bbcdc5d387ceb21ea7458e80a64be72
```

还检查了 `icm45686.c`、`imu_task.c`、`main.c`、`stm32g4xx_it.c` 的预处理有效内容与所有 text section 机器码/relocations，一致。这里不是与未知历史 CubeIDE 版本生成的发布 BIN 比较；ELF 的调试路径信息也不计为运行行为变化。

复现完整 BIN 对比：

```bash
python3 tools/verify_legacy_compatibility.py \
  --toolchain-root "$ARM_GNU_TOOLCHAIN_ROOT" \
  --output build/legacy-compatibility.json
```

见 [原始兼容性结果](legacy-compatibility.json) 和 [构建/校验摘要](timestamp-validation.json)。

## 离线真实源码测试

`python3 tools/test_fifo_unit.py`：真实 ICM 驱动源码 + HAL 寄存器模型，**5/5 ASan/UBSan 通过**，覆盖配置次序与勘误、原 ODR/滤波/量程保持、双读取 packet count、大小端、20 位正负值/饱和、完整温度、非法 header/样本标记、FULL、部分 SPI 失败、有限 flush 超时、reset/config 失效和 MREG 禁区。

`python3 tools/test_timestamp_unit.py`：真实 task 与 clock 源码 + HAL/RTOS 模型，**5/5 ASan/UBSan 通过**，覆盖多次 raw16 回绕、首样本为零的时钟段、无效增量与 uint64 边界、原 16 字节前缀、BE64 源时间、逐帧 epoch/boot、丢弃所有状态帧后仍可区分 epoch、metadata 入队失败、传感器复位恢复、32 帧排空上限和 self-notify。

目标 Arm GCC 对修改的默认关闭模块以及开启后的 task/ICM/FIFO/clock 模块执行 `-fanalyzer -Wall -Wextra -Werror`（允许 HAL 回调未用参数），全部通过。新增 Python 工具的 Ruff F/E9 与 `git diff --check` 通过。

## 尚未证明的部分

没有真实 OpenIMU 连接，因此没有烧录、物理 SPI/FIFO、INT1 门限、CAN 总线、真实硬件时间戳精度、温度/量程换算或故障恢复的实物验收。模型与交叉编译不能代替这些检查。

开启后的数据帧为 32 字节，与以前 C5 网关 16 字节负载性能测试不同；不能将旧吞吐/延迟数据直接作为新格式的实测结果。上板前还需按 [新协议](source-timestamps.md) 更新接收端，保留逐帧源时间及 clock identity，并在时钟段改变时中断积分连续性。
