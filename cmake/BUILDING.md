# Command-line firmware build

CMake 3.21+, Ninja and Arm GNU Toolchain are required. This build keeps the
STM32CubeIDE project intact. The compiler CPU/FPU/ABI settings, source inventory,
include directories and Debug/Release symbols follow `.cproject`:

- STM32G431CBUx, Cortex-M4 Thumb, `fpv4-sp-d16`, hard-float ABI.
- C11 with GNU extensions; `USE_HAL_DRIVER`, `STM32G431xx`,
  `STM32_THREAD_SAFE_STRATEGY=4`; Debug additionally defines `DEBUG`.
- Debug: `-O0 -g3`; Release: `-Os -g0`.
- Original `STM32G431CBUX_FLASH.ld`, newlib-nano and nosys specifications.

The build prints the selected compiler path and version. The validation toolchain
is Arm GNU Toolchain **14.3.Rel1** for `arm-none-eabi`. To select its extracted
installation rather than the first compiler found on `PATH`:

```sh
export ARM_GNU_TOOLCHAIN_ROOT="$HOME/.local/opt/arm-gnu-toolchain/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi"
"$ARM_GNU_TOOLCHAIN_ROOT/bin/arm-none-eabi-gcc" --version
cmake --preset debug-default
cmake --build --preset debug-default
```

Each preset has its own build directory under `build/`. All builds produce
`open_imu.elf`, `open_imu.bin`, `open_imu.hex`, `open_imu.map` and
`compile_commands.json`; object files and compiler stack-usage reports stay in
the same build tree. Building never flashes a device.

## Optional source timestamp switch

`OPENIMU_SOURCE_TIMESTAMP` defaults to `OFF`. It explicitly defines
`IMU_ENABLE_SOURCE_TIMESTAMP=0` for C compilation; enabling it defines `1`.
`Core/Src/imu_timestamp.c` and `Core/Src/icm45686_fifo.c` are compiled only when
`ON`. An enabled build fails at configuration time if either source is missing.
The option does not rewrite `.cproject` or CubeIDE settings.

```sh
# No switch supplied: default OFF.
cmake --preset debug-default
cmake --build --preset debug-default

# Explicit OFF, isolated from the default build.
cmake --preset debug-off
cmake --build --preset debug-off

# Optional timestamp/FIFO path.
cmake --preset debug-on
cmake --build --preset debug-on

# CubeIDE-style size-optimized release, default OFF.
cmake --preset release-default
cmake --build --preset release-default

# Size-optimized release with source timestamps/FIFO enabled.
cmake --preset release-on
cmake --build --preset release-on
```

Presets describe initial cache values. Reusing a build directory preserves its
CMake cache, including any command-line overrides. To re-check a preset's clean
default, use `cmake --fresh --preset debug-default` (CMake 3.24+) or a new build
directory. Do not share one build directory between different toolchains.

Without presets, the same switch can be passed directly:

```sh
cmake -S . -B build/manual -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug -DOPENIMU_SOURCE_TIMESTAMP=ON
cmake --build build/manual
```

## Reproduce default-OFF compatibility

The standard-library-only verifier builds the original upstream commit and the
current source with both default and explicit OFF. It copies the current CMake
definition onto a `git archive` of the baseline, uses the same Arm compiler and
Debug settings for all three isolated builds, and compares the complete flash
BIN byte for byte. Optional FIFO/timestamp files must not be compiled with OFF.
ELF debug paths are excluded from the comparison; this does not compare against
an unknown historical CubeIDE compiler's binary.

```sh
python3 tools/verify_legacy_compatibility.py \
  --toolchain-root "$ARM_GNU_TOOLCHAIN_ROOT" \
  --output build/legacy-compatibility.json
```

The default baseline is upstream commit
`084aacd90565bac838c9c7f650ea869d62d6522b`, which must already exist in the local
Git object database; use `--upstream-commit` to select another reviewed baseline.
The verifier uses temporary directories and does not modify existing build
caches, check out branches, fetch repositories or flash hardware. It requires
Python 3.9+, Git, CMake, Ninja and the Arm toolchain.

A successful cross-build confirms preprocessing, compilation and linking. FIFO
layout, source timing and CAN behavior still require tests with the physical IMU
and receiving hardware.
