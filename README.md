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
| SPI1_CS     | PA4  | ICM-45686 chip-select (software GPIO, active-low) |
| EXTI0 / INT1| PB0  | ICM-45686 data-ready interrupt, rising-edge triggered |
| FDCAN1_TX   | PA12 | CAN FD transmit |
| FDCAN1_RX   | PA11 | CAN FD receive (TX-only in this project, unused) |
| LED         | PB2  | Status LED (current-sinking: LOW = ON) |

SPI runs in **Mode 3** (CPOL = High, CPHA = 2nd edge) with **software chip-select**:
PA4 is a plain GPIO output that the driver drives low around each transfer
(`SPI_NSS_SOFT` + `HAL_GPIO_WritePin`). The STM32 hardware NSS is *not* used —
its automatic pulse timing did not keep CS asserted for the whole burst.

### 1.1 Status LED (PB2)

PB2 drives a status LED in current-sinking configuration (LOW = ON):

| State             | LED behavior                        |
|-------------------|-------------------------------------|
| Power-on / init   | Solid ON                            |
| Normal (CAN TX)   | Toggle every 100 frames (~4 Hz)     |
| Runtime error     | Fast blink (toggle every 30 ms)     |
| Fatal init error  | Fast blink forever                  |

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
| SPI              | Mode 3, software CS (PA4), blocking `HAL_SPI_TransmitReceive` burst | Most deterministic for a 14-byte transfer; software CS avoids hardware-NSS timing quirks |
| CAN              | FD + BRS, 5 Mbit/s data phase                 | Shorter data-phase transmission time |

### 2.2 File Responsibilities

```
Core/Src/icm45686.c        ICM-45686 driver: software CS (PA4) SPI transport,
                           register + MREG access, soft reset, WHO_AM_I check,
                           ODR/FSR/LN-filter config, 14-byte burst read,
                           endianness auto-detection
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

### 3.1 Physical-layer configuration

- **Transmitter Delay Compensation (TDC)** is enabled. With the FDCAN kernel clock
  at 100 MHz one *mtq* is 10 ns, so `TDCO = 16` places the secondary sample point
  about **160 ns** into the data bit. Without TDC — or with `TDCO = 0` — the
  5 Mbit/s data phase fails with bit errors even though the 1 Mbit/s arbitration
  phase still looks perfect on a scope. This is what made 16-byte FD frames
  transmit reliably.
- **Bus-off auto-recovery**: with no ACK (no receiver on the bus) the node
  eventually enters bus-off. The task watches `PSR.BO` and clears `CCCR.INIT`, so
  the node recovers and keeps retrying. A receiver can be connected at any time;
  power-up order does not matter.

### 3.2 Frame example

```
ID 0x001  [16]  30 F5 02 65 F1 8C 1C AE 00 0D 00 03 00 08 01 00
           seq   | accel X/Y/Z (int16 BE) | gyro X/Y/Z (int16 BE) | temp
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

> The complete frame (16 data bytes + CRC + ACK/EOF) is on the bus ~70-80 us after transmission starts.

### 4.2 Total End-to-End Latency

From "sample ready inside the sensor" to "the CAN frame is fully sent on the bus":

```
total latency ~= sensor group delay (1-3 ms) + MCU path (~15-20 us) + full CAN FD frame (~70-80 us)
              ~= sensor group delay + ~85-100 us
```

**Conclusion**: the MCU + CAN path adds only ~85-100 us on top of the sensor's internal group delay; **the real bottleneck is the sensor's internal filter group delay (millisecond scale)**. To reduce the overall latency, start with the sensor filter bandwidth (section 5.2).

### 4.3 Jitter and Frame Loss

- The sample period is 800 Hz = 1.25 ms, while the whole MCU+CAN path (up to a complete frame on the bus) is ~85-100 us, only **~7-8%** of the period. CPU and bus are far from saturated, so **jitter is small and there is no backlog**.
- `imu_task` is woken via task notification. If a higher-priority task briefly blocks it and one ready pulse is missed, `ulTaskNotifyTake` returns >1 and `imu_missed_count` increments; the task still reads the **latest** register sample, so **no latency is added** - only the intermediate sample is lost (the receiver can detect the gap via the `seq` counter).
- If CAN bus congestion fills the 3-deep TX FIFO, `HAL_FDCAN_AddMessageToTxFifoQ` returns an error and `imu_can_error_count` increments; the frame is dropped (the task is never blocked, and latency does not accumulate).

### 4.4 Diagnostic Counters (observe in the debugger)

| Variable | Meaning |
|---|---|
| `imu_sample_count` | Frames successfully queued to the TX FIFO |
| `imu_missed_count` | Data-ready overruns (task failed to respond in time) |
| `imu_can_error_count` | CAN TX FIFO full / enqueue failures |
| `imu_spi_error_count` | SPI read failures |
| `can_status` (`FDCAN_ProtocolStatusTypeDef`) | Protocol status: `BusOff`, `ErrorPassive`, `LastErrorCode`, `DataLastErrorCode` |
| `can_errors` (`FDCAN_ErrorCountersTypeDef`) | `TxErrorCnt` / `RxErrorCnt`; both should stay 0 on a clean bus |

> `imu_sample_count` only proves a frame was *enqueued*, not that it reached the
> bus. Bus/bit errors happen after enqueue, so check `can_errors.TxErrorCnt`
> (should stay 0) and `can_status.LastErrorCode` to confirm real transmission.

---

## 5. Further Latency-Reduction Options

### 5.1 Already Applied (implemented in code)

- Interrupt-driven + task-notification wakeup (instead of polling / semaphore).
- Direct read of the latest register sample (instead of bulk FIFO reads).
- FDCAN FD + BRS, 5 Mbit/s data phase.

### 5.2 Sensor: UI low-pass filter bandwidth (highest return)

The ICM-45686 gyro/accel UI low-pass filter bandwidth is configurable; the wider
the bandwidth, the lower the group delay (at the cost of more noise). The driver
already sets both filters to **ODR/4** (`ICM45686_UI_LPFBW_DIV_4`, written through
MREG `0xA4AC` / `0xA583` during init).

| Sensor | MREG address | Field | Description |
|---|---|---|---|
| Gyro | `IPREG_SYS1_REG_172` = `0xA4AC` | `gyro_ui_lpfbw_sel[2:0]` | `0` = no filter (widest / lowest latency) ... `6` = narrowest |
| Accel | `IPREG_SYS2_REG_131` = `0xA583` | `accel_ui_lpfbw_sel[2:0]` | same as above |

For the absolute lowest latency set both to `0` (NO_FILTER) in `ICM45686_Init()`;
this trades noise for group delay. Write them the same way as `SREG_CTRL`: 16-bit
address to `IREG_ADDR_15_8(0x7C)` / `IREG_ADDR_7_0(0x7D)`, then the data byte to
`IREG_DATA(0x7E)`.

### 5.3 SPI Clock

Currently 12.5 MHz (24 MHz is the ICM-45686 upper limit; margin was kept). It can be changed to `SPI_BAUDRATEPRESCALER_4` (25 MHz, slightly over spec) or a clock source closer to 24 MHz, reducing the 14-byte burst read from ~10 us to ~5 us. The gain is relatively small (<10 us).

### 5.4 CAN Bit Rate

- The 1 Mbit/s arbitration rate is the industry standard; raising it to 2 Mbit/s cuts the arbitration phase from ~27 us to ~14 us, but reduces compatibility with ordinary 1 Mbit/s nodes.
- The 5 Mbit/s data phase can be raised to 8 Mbit/s (STM32G4 FDCAN limit), shortening the data phase by ~10 us.

> Overall, **the sensor's internal filter group delay (millisecond scale) is the real dominant term**; the microsecond-scale MCU/CAN optimizations have limited impact on total latency. Apply section 5.2 first for the highest return.

---

## 6. Build & Flash

1. Open the project in STM32CubeIDE and run `Project -> Build` (verified with a full build using the 13.3 toolchain).
2. After flashing, connect at least one receiver that supports **CAN FD + BRS with a data phase >= 5 Mbit/s**, and place **120 ohm termination resistors** at both ends of the bus.
3. If the node is alone on the bus (no ACK) it falls back to bus-off but **auto-recovers**: the task clears `CCCR.INIT` whenever `PSR.BO` is set, so the node keeps retrying and starts streaming as soon as a receiver is connected. Power-up order does not matter.

> Note: regenerating code with CubeMX overwrites the SPI/FDCAN init in the generated region of `main.c`; re-apply SPI = **8-bit / Mode 3 (CPOL High, CPHA 2Edge) / 12.5 MHz / NSS soft**, with **PA4 as a GPIO output** (software CS, idle high), and FDCAN = FD+BRS (1M/5M) plus the runtime TDC / bus-off handling in `imu_task.c`.

---

## 7. Receiving the Data on Linux (candump)

The board sends CAN FD frames: **ID `0x01`, 16 bytes, nominal 1 Mbit/s, data phase 5 Mbit/s (BRS)**.
On Linux, use a CAN FD-capable SocketCAN adapter (candleLight/gs_usb, PCAN-USB FD, Kvaser, ...).

### 7.1 Bring up the interface

```bash
# Replace can0 with your adapter's interface name (`ip link` lists them)
sudo ip link set can0 down
# CAN FD: nominal 1 Mbit/s, data phase 5 Mbit/s, BRS enabled
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

Verify:

```bash
ip -details link show can0
```

It should report `bitrate 1000000 ... dbitrate 5000000 ... fd on`.

> Some drivers reject `fd on` without a data bitrate (`Error: Provide both nominal and FD data bittiming`), so always pass both `bitrate` and `dbitrate`.
> A classic-CAN-only adapter cannot receive these frames.

### 7.2 Dump the frames

```bash
candump -d can0          # payload printed as hex
candump -d -t d can0     # + delta timestamps (useful to spot dropped frames)
```

One frame arrives every 1.25 ms (800 Hz):

```
can0  001  [16]  30 F5 02 65 F1 8C 1C AE 00 0D 00 03 00 08 01 00
```

- `001` -> CAN ID `0x01`
- `[16]` -> 16 data bytes (DLC code 10)
- the payload layout is in section 3

### 7.3 Decode one frame

The 16 bytes are big-endian: `seq[0:1]`, `accel X/Y/Z[2:7]`, `gyro X/Y/Z[8:13]`, `temp[14:15]`.

```python
import struct
b = bytes.fromhex("30 F5 02 65 F1 8C 1C AE 00 0D 00 03 00 08 01 00")
seq = int.from_bytes(b[0:2], "big")
acc = struct.unpack(">3h", b[2:8])    # raw accel X/Y/Z
gyr = struct.unpack(">3h", b[8:14])   # raw gyro  X/Y/Z
tmp = struct.unpack(">h", b[14:16])[0]
print(seq, [a / 8192.0 for a in acc], [g / 16.4 for g in gyr], tmp / 132.48 + 25)
```

---

## 8. Python AHRS Demo (Mahony / Madgwick / SixAxis)

`tools/ahrs_demo.py` reads the 16-byte CAN FD stream and runs three attitude
estimators on the same data, so they can be compared live:

| Estimator | Notes |
|---|---|
| **Mahony** | nonlinear complementary filter, proportional + integral correction |
| **Madgwick** | gradient-descent AHRS (`--madgwick-beta`) |
| **SixAxis** | port of BeijingDynamics `open_sprite_runtime` `native/src/imu_six_axis.hpp`: proportional correction with a startup static gyro bias and an accelerometer trust window (no magnetometer) |

It shows a 3-D attitude view, the roll/pitch/yaw comparison and the world-frame
linear ("motion") acceleration. Yaw cannot be observed from the accelerometer,
so on a static board its **slope is exactly the residual gyro bias** - use the
bottom yaw plot (and the `yaw ... (deg/s)` readout) to assess static drift.

### 8.1 Install

```bash
pip install python-can numpy matplotlib
```

### 8.2 Run

```bash
# Linux: SocketCAN (configure the interface first, see section 7)
python tools/ahrs_demo.py -i socketcan -c can0

# Windows: PCAN-USB FD via PCAN-Basic
python tools/ahrs_demo.py -i pcan -c PCAN_USBBUS1

# backend auto-detected (socketcan on Linux, pcan on Windows)
python tools/ahrs_demo.py

# console only, no window (quick check)
python tools/ahrs_demo.py --no-plot --duration 10

# UI/algorithm test without hardware
python tools/ahrs_demo.py --simulate
```

Keep the board **still during the first `--calib-seconds`** (default 2 s): the
demo estimates the gyro bias from that window and initialises all three filters.

Options: `--mahony-kp`, `--mahony-ki`, `--madgwick-beta`, `--calib-seconds`,
`--history`, `--log samples.csv`, `--f-clock` (PCAN clock, default 80 MHz).

`tools/gyro_dead_reckoning.py` is a companion demo that removes the
accelerometer correction entirely: a 10 s static calibration (gyro bias +
roll/pitch from gravity, yaw = 0), then **gyro-only** quaternion integration.
The resulting attitude rotates the accelerometer into the world frame, gravity
is subtracted, and the acceleration is integrated once for **velocity** and once
more for **displacement** (both drift open-loop; the plot shows the X-Y drift
path):

```bash
python tools/gyro_dead_reckoning.py -i pcan -c PCAN_USBBUS1
```

---

## 9. 3D-Printed Enclosure (FreeCAD)

The `3D/` folder contains the FreeCAD models: the PCB model, two enclosure parts,
and the full assembly.

![3D preview](3D/open_imu.png)

See [3D/README.md](3D/README.md) for the file list.

**Printing recommendation: PETG.** Compared with PLA it is tougher, warps far
less and has better layer adhesion, which suits this small enclosure with
mounting ears. Suggested profile: 240-250 C nozzle / 80 C bed, 3-4 perimeters,
25-40 % infill, 0.16-0.20 mm layers.
