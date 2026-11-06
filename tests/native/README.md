# Native firmware fault model

Run from the repository root:

```sh
python3 tools/validate_clock_sync.py
```

The tests include the actual `Core/Src/imu_sync.c`, compiled as C with
`-Wall -Wextra -Werror -fsanitize=address,undefined`. They do not duplicate the
firmware state machine in a second implementation. HAL calls are replaced by
controlled stubs in `sync_mock.h`; register-access seams default to direct
register access in production.

The register model implements the effects relevant to these cases: TIM3 status
bits clear on writing zero, reading CCR3 acknowledges CC3IF, and CAN RX-lost
status clears on writing one. Hooks inject overflow/capture arrivals between
register accesses and a 66 ms task preemption inside a CAN FIFO read. Each fault
scenario runs in a fresh process to avoid hidden state inherited from another
test. Initialization retries are exercised explicitly within one scenario.

The 21 fault scenarios cover four CAN configuration failure stages, RNG timeout,
failed reinitialization, RX preemption/continuous read failures, RX overflow
during reading and before reporting, failed/blocked revocation transmission,
counter-read races, capture arrival during an ISR, capture around counter wrap,
stale capture after one wrap, first-IRQ missed wraps, capture reuse/overrun,
HAL tick wrap at lease expiry, malformed controls, and source timestamp rollover.

The two-hour source test feeds 5,760,000 synthetic 800 Hz samples through the
actual C source-time expansion and DATA extension functions. It checks every
sample against an independent 64-bit elapsed-time calculation and verifies that
the first 16 payload bytes remain unchanged. Timer servicing is represented by
advancing the mock clock; this is not an execution-time, interrupt-throughput,
or full acquisition-task simulation.

The original native suite separately exports valid and revoked ANCHOR bytes;
the Python validator decodes those bytes with the real host decoder. This catches
wire-layout disagreements between the two implementations.

The mocks do not execute STM32 silicon, the HAL implementation, the FreeRTOS
scheduler, SPI transactions, or a physical CAN controller. GPIO alternate-function
routing, electrical edges, oscillator behavior, FIFO/INT timing and actual bus
arbitration still need hardware verification. Modeled register behavior provides
regression evidence for the explicit interleavings above, not an exhaustive proof
of all possible peripheral interleavings.

## Acquisition-task integration

`task_integration_test.c` also includes the actual `imu_task.c` and `imu_sync.c`.
A bounded sequence of mocked task notifications drives the real acquisition loop;
mocked ICM reads and HAL enqueue calls expose sample content and ordering. Three
scenarios cover dormant/START/STOP transitions, FIFO backlog and SPI-read failure.
The TX hook asserts DATA is inserted before its ANCHOR. Replaying the previous
anchor-first task implementation fails this assertion. This is a task-level
integration test, not a FreeRTOS scheduler or physical SPI/CAN simulation.
