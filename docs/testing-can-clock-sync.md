# Clock synchronization validation

## Reproduce software checks

```sh
python3 tools/validate_clock_sync.py
OPENIMU_LONG_SIM=1 python3 -m unittest discover -s tools -p test_clock_sync.py -v
python3 cmake/validate_builds.py --report build/validation-builds.json
```

The build checker needs Arm GNU GCC (PATH or `ARM_GNU_TOOLCHAIN_ROOT`), CMake and
Ninja. It builds 11 configurations covering all five enabled node IDs and both
Debug/Release modes. Optional `--baseline-dir PATH` compares disabled node-1 BINs
against existing upstream-source `debug/open_imu.bin` and `release/open_imu.bin`
files compiled with the same compiler and flags.

Native tests require a C compiler with ASan/UBSan support (tested on Linux). They
execute the real sync module with register effects and injected timing failures.
The acquisition integration test executes the actual `IMU_Task` with mocked SPI,
notifications and CAN queue calls, checking dormant/START/STOP formats, sample-
before-anchor insertion order, FIFO backlog invalidation and SPI-read failure.
It does not implement a complete FreeRTOS scheduler or physical CAN arbitration.
See [the mock model](../tests/native/README.md).

The optional two-hour, five-node, 800 Hz model checks every accepted timestamp
against synthetic truth, along with monotonicity, bounded histories, dropped and
reordered packets, resets and synchronization outages. Simulation duration is not
wall-clock hardware endurance, and its error bound is not a hardware accuracy
specification.

Static analysis uses `-fanalyzer -Werror`. The unchanged upstream fatal-loop files
listed in `validate_builds.py` suppress only `-Wanalyzer-infinite-loop`; the new
sync module has no such exception. Compiler/ASan success is not proof of interrupt
or peripheral timing correctness.

## Hardware acceptance still required

- [ ] Verify enabled-but-dormant sampling and the existing dashboard on a real board.
- [ ] Check PB0 AF2 capture and EXTI coexistence and the exact INT1/FIFO pairing.
- [ ] Verify FDCAN RX SOF and TIM3 share the expected timebase across rollover.
- [ ] Exercise START/STOP, host loss, sensor restart, reference restart and bus-off.
- [ ] Measure worst-case interrupt latency, full task-stack usage and TX ordering.
- [ ] Run five nodes at 800 Hz for at least two hours; quantify loss and jitter.
- [ ] Measure actual cross-node timing error independently of fitted residuals.
- [ ] Validate downstream transport separately when integrating a C5/MQTT bridge.

This feature stays experimental and disabled by default pending those checks.
