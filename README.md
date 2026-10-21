# open_imu — ICM-45686 → CAN FD Real-Time Data Stream

An IMU data-acquisition and CAN FD transmission project based on the **STM32G431CBU6** + **ICM-45686** (6-axis gyro + accelerometer, SPI interface). The sensor outputs samples at **800 Hz ODR**, streamed in real time over **CAN FD (standard frame, ID = 0x01)**. The system runs FreeRTOS.

Design goal: **the lowest possible end-to-end latency**.

---

## 1. Hardware Connections

| Peripheral  | Pin  | Description |
|-------------|------|-------------|
| SPI1_SCK    | PA5  | ICM-45686 SCK, 12.5 MHz |
| SPI1_MISO   | PA6  | ICM-45686 SDO |
| SPI1_MOSI   | PA7  | ICM-45686 SDI |
| SPI1_NSS    | PA4  | ICM-45686 CS (hardware NSS) |
| EXTI0 / INT1| PB0  | ICM-45686 data-ready interrupt, rising-edge triggered |
| FDCAN1_TX   | PA12 | CAN FD transmit |
| FDCAN1_RX   | PA11 | CAN FD receive (TX-only in this project, unused) |

---

## 2. Data-Flow Architecture

The system uses an **"interrupt-driven + read-register"** model rather than polling or bulk FIFO reads, so every CAN frame closely follows the sensor output and only the **latest** sample is sent.

```
                    ┌──────────────────────────────────────────────┐
                    │                 ICM-45686                    │
                    │   accel(3) + gyro(3) @ 800 Hz ODR, LN mode   │
                    │   sample ready -> INT1 asserts a high pulse  │
                    └───────────────┬──────────────────────────────┘
                                    │ INT1 (800 Hz, 1.25 ms period)
                                    v
                    ┌──────────────────────────────────────────────┐
                    │   PB0 = EXTI0 (rising edge)                  │
                    │   EXTI0_IRQHandler -> HAL_GPIO_EXTI_IRQHandler│
                    │        -> HAL_GPIO_EXTI_Callback()           │
                    │        -> IMU_DataReadyFromISR()             │
                    │        -> vTaskNotifyGiveFromISR() (zero-heap)│
                    └───────────────┬──────────────────────────────┘
                                    │ direct wakeup (PendSV context switch)
                                    v
                    ┌──────────────────────────────────────────────┐
                    │   imuTask (priority osPriorityHigh)          │
                    │   blocks on ulTaskNotifyTake() -> woken up   │
                    │   1) ICM45686_ReadData()  -- 14-byte SPI burst│
                    │   2) IMU_PackFrame()      -- 16-byte big-endian frame │
                    │   3) HAL_FDCAN_AddMessageToTxFifoQ() -- TX FIFO│
                    └───────────────┬──────────────────────────────┘
                                    │
                                    v
                    ┌──────────────────────────────────────────────┐
                    │   FDCAN1 hardware (1 Mbit/s arbitration +    │
                    │   5 Mbit/s data phase, BRS)                  │
                    │   automatic arbitration, bit-stuffing, CRC, TX│
                    └──────────────────────────────────────────────┘
```

### 2.1 Key Design Choices

| Concern          | Choice                                        | Latency benefit |
|------------------|-----------------------------------------------|-----------------|
| Sample trigger   | INT1 data-ready interrupt                     | Exactly synchronized to the sensor 800 Hz output; no polling idle time |
| ISR -> task wake | `vTaskNotifyGiveFromISR` / `ulTaskNotifyTake` | Fastest FreeRTOS wake primitive; no queue/semaphore object or memory overhead |
| Read method      | Read data registers directly (not the FIFO)   | Always reads the **latest** sample; no FIFO backlog latency |
| Task priority    | `osPriorityHigh`                              | Scheduled almost immediately after wakeup |
| SPI              | Blocking `HAL_SPI_TransmitReceive` burst read | Most deterministic for a 14-byte transfer; no DMA setup overhead |
| CAN              | FD + BRS, 5 Mbit/s data phase                 | Shorter data-phase transmission time |

### 2.2 File Responsibilities

```
Core/Src/icm45686.c        ICM-45686 driver: SPI transport, register access, soft
                           reset, WHO_AM_I check, ODR/FSR config, 14-byte burst
                           read, endianness auto-detection
Core/Inc/icm45686.h        Register addresses, ODR/FSR enums, data structure, scale factors
Core/Src/imu_task.c        FreeRTOS task: init, EXTI0 enable, frame packing, FDCAN TX,
                           diagnostic counters
Core/Inc/imu_task.h        Task interface, CAN ID, frame length
Core/Src/main.c            SPI/FDCAN peripheral init (fixed to 8-bit / 12.5 MHz / FD+BRS)
Core/Src/stm32g4xx_it.c    EXTI0_IRQHandler
```

---

## 3. CAN FD Frame Format

- **ID**: `0x01` (standard frame)
- **Type**: data frame, FD format, BRS enabled (1 Mbit/s arbitration / 5 Mbit/s data phase)
- **Length**: 16 bytes; multi-byte fields are **big-endian**

```
Byte(s)    Content
[0:1]      seq          frame sequence number (uint16, for dropped-frame detection)
[2:3]      accel X      raw int16
[4:5]      accel Y
[6:7]      accel Z
[8:9]      gyro X       raw int16
[10:11]    gyro Y
[12:13]    gyro Z
[14:15]    temperature  raw int16
```

**Receiver-side conversion** (current ranges +/-4 g / +/-2000 dps):

```
accel_g  = raw_accel / 8192.0
gyro_dps = raw_gyro  / 16.4
temp_degC = raw_temp / 132.48 + 25
```

---

## 4. Latency Analysis

### 4.1 Latency Composition

End-to-end latency = **sensor internal latency** + **MCU processing path** + **CAN transmission path**.

#### (1) Sensor internal latency (dominant term)

In low-noise mode, the ICM-45686 has an internal ADC-conversion and UI low-pass (anti-aliasing) filter chain. This **group delay** is typically the largest component of the whole path, on the order of **1-3 ms** (depending on the internal UI low-pass filter bandwidth setting; see section 5.2). It is determined by the MEMS sensor itself, independent of the MCU, and is the inherent cost of "physical quantity -> digital sample ready".

#### (2) MCU processing path (INT1 edge -> data queued in CAN TX FIFO)

| Stage | Typical time | Notes |
|---|---|---|
| INT1 edge -> NVIC enters ISR | ~0.2-0.5 us | Cortex-M4 interrupt response ~12 cycles + EXTI sync (100 MHz) |
| Notify task from ISR | ~0.5-1 us | HAL callback + `vTaskNotifyGiveFromISR` |
| PendSV context switch to imuTask | ~1-2 us | FreeRTOS Cortex-M4F context switch (incl. lazy FPU stacking) |
| 14-byte SPI burst read | ~10-12 us | 15 bytes x 8 bit @ 12.5 MHz ~= 9.6 us + HAL overhead |
| Pack + enqueue to FDCAN TX FIFO | ~2-3 us | packing + `HAL_FDCAN_AddMessageToTxFifoQ` |

**Total MCU path ~= 15-20 us.**

#### (3) CAN FD transmission path (queued -> on the bus)

A 16-byte FD frame (1 Mbit/s arbitration + 5 Mbit/s data phase, incl. bit-stuffing / CRC / EOF / IFS):

| Stage | Typical time |
|---|---|
| Arbitration phase (SOF+ID+FDF+BRS+ESI+DLC, ~22 bits + stuffing) @ 1 Mbit/s | ~25-30 us |
| Data phase (128 bits + stuffing + 21-bit CRC) @ 5 Mbit/s | ~35 us |
| Trailer (ACK + EOF + IFS) @ 1 Mbit/s | ~13 us |
| **Full frame total** | **~70-80 us** |

> The first data byte appears on the bus right after the arbitration phase ends, i.e. ~25-30 us after transmission starts.

### 4.2 Total End-to-End Latency

From "sample ready inside the sensor" to "receiver gets the first data byte":

```
total latency ~= sensor group delay (1-3 ms) + MCU path (~15-20 us) + CAN arbitration phase (~25-30 us)
              ~= sensor group delay + ~40-50 us
```

**Conclusion**: the extra latency introduced by the MCU + CAN is only ~40-50 us; **the real bottleneck is the sensor's internal filter group delay**. To reduce the overall latency, start with the sensor filter bandwidth (section 5.2).

### 4.3 Jitter and Frame Loss

- The sample period is 800 Hz = 1.25 ms, while the whole MCU+CAN path is ~90 us, only **~7%** of the period. CPU and bus are far from saturated, so **jitter is small and there is no backlog**.
- `imu_task` is woken via task notification. If a higher-priority task briefly blocks it and one ready pulse is missed, `ulTaskNotifyTake` returns >1 and `imu_missed_count` increments; the task still reads the **latest** register sample, so **no latency is added** - only the intermediate sample is lost (the receiver can detect the gap via the `seq` counter).
- If CAN bus congestion fills the 3-deep TX FIFO, `HAL_FDCAN_AddMessageToTxFifoQ` returns an error and `imu_can_error_count` increments; the frame is dropped (the task is never blocked, and latency does not accumulate).

### 4.4 Diagnostic Counters (observe in the debugger)

| Variable | Meaning |
|---|---|
| `imu_sample_count` | Frames successfully queued to the TX FIFO |
| `imu_missed_count` | Data-ready overruns (task failed to respond in time) |
| `imu_can_error_count` | CAN TX FIFO full / TX failures |
| `imu_spi_error_count` | SPI read failures |

---

## 5. Further Latency-Reduction Options

### 5.1 Already Applied (implemented in code)

- Interrupt-driven + task-notification wakeup (instead of polling / semaphore).
- Direct read of the latest register sample (instead of bulk FIFO reads).
- FDCAN FD + BRS, 5 Mbit/s data phase.

### 5.2 Sensor: widen the internal UI low-pass filter (highest return)

The ICM-45686 gyro/accel UI low-pass filter bandwidth is configurable; the wider the bandwidth, the lower the group delay (at the cost of more noise). The current driver keeps the factory default; for the lowest latency, add the following to `ICM45686_Init()` in `icm45686.c` (via indirect MREG writes):

| Sensor | MREG address | Field | Description |
|---|---|---|---|
| Gyro | `IPREG_SYS1_REG_172` = `0xA4AC` | `gyro_ui_lpfbw_sel[2:0]` | `0` = no filter (widest / lowest latency) ... `6` = narrowest |
| Accel | `IPREG_SYS2_REG_131` = `0xA583` | `accel_ui_lpfbw_sel[2:0]` | same as above |

Write them the same way as `SREG_CTRL`: first write the 16-bit address to `IREG_ADDR_15_8(0x7C)` / `IREG_ADDR_7_0(0x7D)`, then write the data to `IREG_DATA(0x7E)`. Setting `0` (NO_FILTER) gives the lowest group delay.

### 5.3 SPI Clock

Currently 12.5 MHz (24 MHz is the ICM-45686 upper limit; margin was kept). It can be changed to `SPI_BAUDRATEPRESCALER_4` (25 MHz, slightly over spec) or a clock source closer to 24 MHz, reducing the 14-byte burst read from ~10 us to ~5 us. The gain is relatively small (<10 us).

### 5.4 CAN Bit Rate

- The 1 Mbit/s arbitration rate is the industry standard; raising it to 2 Mbit/s cuts the arbitration phase from ~27 us to ~14 us, but reduces compatibility with ordinary 1 Mbit/s nodes.
- The 5 Mbit/s data phase can be raised to 8 Mbit/s (STM32G4 FDCAN limit), shortening the data phase by ~10 us.

> Overall, **the sensor's internal filter group delay (millisecond scale) is the real dominant term**; the microsecond-scale MCU/CAN optimizations have limited impact on total latency. Apply section 5.2 first for the highest return.

---

## 6. Build & Flash

1. Open the project in STM32CubeIDE and run `Project -> Build` (verified with a full build using the 13.3 toolchain).
2. After flashing, if the node is alone on the bus (no other CAN node to acknowledge), transmission fails due to missing ACK (`imu_can_error_count` increments) - this is normal bus behavior. For real testing, connect at least one receiver that supports **CAN FD + BRS with a data phase >= 5 Mbit/s**, and place **120 ohm termination resistors** at both ends of the bus.

> Note: regenerating code with CubeMX overwrites the SPI/FDCAN init in the generated region of `main.c`; re-apply SPI = 8-bit / 12.5 MHz / NSS pulse off, and FDCAN = FD+BRS (1M/5M).
