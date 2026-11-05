#!/usr/bin/env python3
"""Exercise the real optional FIFO driver and SPI transport using a host HAL model.

The model checks configuration order and SPI burst boundaries; tests cover the
TDK register map, count erratum, FIFO20 endianness/scaling, and failure recovery.
No hardware is accessed. Requires a host C compiler with ASan/UBSan support.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent

HAL_STUB = r'''
#ifndef UNIT_MAIN_H
#define UNIT_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { int unused; } SPI_HandleTypeDef;
typedef enum {HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3} HAL_StatusTypeDef;
typedef enum {GPIO_PIN_RESET=0, GPIO_PIN_SET=1} GPIO_PinState;
#define GPIOA ((void *)1)
#define GPIO_PIN_4 16U
#define HAL_MAX_DELAY UINT32_MAX
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState value);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout);
void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);
#endif
'''

HARNESS = r'''
#include "icm45686_fifo.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
static uint8_t regs[128], mreg[65536], fifo[16][20];
static uint16_t mreg_address;
static unsigned fifo_count, fifo_read, fifo_write, count_reads, fifo_bursts;
static unsigned spi_calls, source_only, cs = 1, flush_auto = 1, force_count_first_zero;
static int fail_reg = -1, mismatch_reg = -1;
static unsigned partial_failure;
static uint32_t ticks;
static SPI_HandleTypeDef spi;
struct write { uint8_t reg, value; };
static struct write writes[1024];
static unsigned write_count;
static unsigned fifo_live;

static void defaults(void) {
    memset(regs, 0, sizeof(regs)); memset(mreg, 0, sizeof(mreg));
    regs[0x72] = 0xE9; regs[0x19] = 0x80;
    fifo_count = fifo_read = fifo_write = fifo_live = 0;
}
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState value) {
    assert(port == GPIOA && pin == GPIO_PIN_4);
    if (value == GPIO_PIN_RESET) { assert(cs == 1); cs = 0; }
    else { assert(cs == 0); cs = 1; }
}
void HAL_Delay(uint32_t ms) { ticks += ms; }
uint32_t HAL_GetTick(void) { return ticks; }
static uint16_t count_value(void) {
    count_reads++;
    if (force_count_first_zero && count_reads % 2) return 0;
    return (uint16_t)fifo_count;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *device, uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout) {
    assert(device == &spi && cs == 0 && len >= 2 && len <= 21);
    assert(timeout == (source_only ? 20U : HAL_MAX_DELAY));
    spi_calls++; memset(rx, 0, len); unsigned address = tx[0] & 0x7f;
    if ((int)address == fail_reg) {
        if (address == 0x14 && partial_failure && fifo_count) { fifo_read = (fifo_read + 1) % 16; fifo_count--; }
        return HAL_TIMEOUT;
    }
    if (tx[0] & 0x80) {
        if (address == 0x14) {
            assert(fifo_live && len == 21 && fifo_count > 0);
            memcpy(rx + 1, fifo[fifo_read], 20); fifo_read = (fifo_read + 1) % 16; fifo_count--; fifo_bursts++;
        } else if (address == 0x12) {
            assert(len == 3); uint16_t count = count_value();
            if (mreg[0xA267] & 2) { rx[1] = (uint8_t)(count >> 8); rx[2] = (uint8_t)count; }
            else { rx[1] = (uint8_t)count; rx[2] = (uint8_t)(count >> 8); }
        } else if (address == 0x7e) {
            assert(len == 2); rx[1] = mreg[mreg_address++];
        } else {
            for (unsigned i = 1; i < len; i++) rx[i] = regs[address + i - 1];
            if (address == 0x19) regs[0x19] = 0;
        }
        if ((int)address == mismatch_reg) rx[1] ^= 0x20;
    } else if (address == 0x7c) {
        assert(len == 3 || len == 4); mreg_address = (uint16_t)((unsigned)tx[1] << 8 | tx[2]);
        if (len == 4) mreg[mreg_address++] = tx[3];
    } else {
        for (unsigned i = 1; i < len; i++) {
            unsigned reg = address + i - 1; uint8_t value = tx[i];
            assert(reg < 128); assert(write_count < 1024); writes[write_count++] = (struct write){(uint8_t)reg, value};
            if (reg == 0x7f && value == 2) { defaults(); continue; }
            /* FIFO format/threshold changes require bypass. Interface enable
             * and recovery flush are intentionally allowed while active. */
            if (reg == 0x1e || reg == 0x1f || reg == 0x22 ||
                (reg == 0x20 && ((value ^ regs[reg]) & 8)) ||
                (reg == 0x21 && ((value ^ regs[reg]) & 0x3e)))
                assert((regs[0x1d] & 0xc0) == 0);
            regs[reg] = value;
            if (reg == 0x20 && (value & 0x80) && flush_auto) {
                regs[0x20] &= 0x7f; fifo_count = fifo_read = fifo_write = 0;
            }
            if (reg == 0x21) fifo_live = value & 1;
        }
    }
    return HAL_OK;
}
static void initialize(void) {
    source_only = 0; fail_reg = mismatch_reg = -1; flush_auto = 1; partial_failure = 0;
    assert(ICM45686_Init(&spi) == HAL_OK); source_only = 1;
}
static void enable(unsigned big_endian) {
    initialize(); mreg[0xA267] = big_endian ? 2 : 0;
    /* Deliberately begin with unwanted delta/resolution/FSYNC/compression. */
    regs[0x23] = 0x60; regs[0x24] = 7; mreg[0xA258] = 2;
    regs[0x22] = 0x3c;
    write_count = 0; assert(ICM45686_FifoEnable() == ICM45686_FIFO_OK);
    assert(cs == 1 && regs[0x1d] == 0x9e && regs[0x1e] == 1 && regs[0x1f] == 0);
    assert(regs[0x20] == 8 && regs[0x21] == 15 && regs[0x22] == 2);
    assert((regs[0x23] & 0x60) == 0 && (regs[0x24] & 7) == 0 && (mreg[0xA258] & 3) == 1);
    assert(regs[0x28] == 0 && regs[0x16] == 3 && regs[0x18] == 1);
    assert(regs[0x1b] == 0x36 && regs[0x1c] == 0x16 && regs[0x10] == 15);
    assert((mreg[0xA583] & 7) == 1 && (mreg[0xA4AC] & 7) == 1);
    unsigned low = UINT_MAX, high = UINT_MAX;
    for (unsigned i = 0; i < write_count; i++) {
        if (writes[i].reg == 0x1e) low = i;
        if (writes[i].reg == 0x1f) high = i;
    }
    assert(low != UINT_MAX && high == low + 1);
}
static void put16(uint8_t *p, uint16_t value, unsigned big) {
    p[big ? 1 : 0] = (uint8_t)value; p[big ? 0 : 1] = (uint8_t)(value >> 8);
}
static void push(const int32_t accel[3], const int32_t gyro[3], int16_t temp, uint16_t ts, uint8_t header) {
    assert(fifo_count < 16); uint8_t *p = fifo[fifo_write]; memset(p, 0, 20); p[0] = header;
    unsigned big = !!(mreg[0xA267] & 2);
    for (unsigned i = 0; i < 3; i++) {
        uint32_t a = (uint32_t)accel[i] & 0xfffff, g = (uint32_t)gyro[i] & 0xfffff;
        put16(p + 1 + i * 2, (uint16_t)(a >> 4), big); put16(p + 7 + i * 2, (uint16_t)(g >> 4), big);
        p[17 + i] = (uint8_t)((a & 15) << 4 | (g & 15));
    }
    put16(p + 13, (uint16_t)temp, big); put16(p + 15, ts, big);
    fifo_write = (fifo_write + 1) % 16; fifo_count++;
}
static void unchanged_failure(ICM45686_FifoResult_t expected) {
    ICM45686_FifoSample_t out, before; memset(&out, 0xa5, sizeof(out)); before = out;
    assert(ICM45686_FifoRead(&out) == expected); assert(!memcmp(&out, &before, sizeof(out))); assert(cs == 1);
}
int main(void) {
    uint16_t count = 99; ICM45686_FifoSample_t out;
    assert(ICM45686_FifoGetCount(&count) == ICM45686_FIFO_CONFIG_ERROR && count == 0);
    assert(ICM45686_FifoRead(NULL) == ICM45686_FIFO_CONFIG_ERROR);
    uint8_t scratch[21];
    assert(ICM45686_SourceReadRegisters(0, scratch, 20) == HAL_ERROR);
    enable(0); count = 99; count_reads = 0;
    assert(ICM45686_FifoGetCount(&count) == ICM45686_FIFO_EMPTY && count == 0 && count_reads == 2);
    unchanged_failure(ICM45686_FIFO_EMPTY);
    puts("PASS verified FIFO20 snapshot configuration, WM low/high order, preserved ODR/LPF/range and disabled delta/FSYNC");
    const int32_t a[3] = {-31,-1,524287}, g[3] = {-31,7,524287};
    const int32_t b[3] = {-524272,0,100}, h[3] = {-524272,0,80};
    for (unsigned big = 0; big < 2; big++) {
        enable(big); count_reads = 0; force_count_first_zero = 1;
        push(a,g,-12345,65500,0x78);
        assert(ICM45686_FifoRead(&out) == ICM45686_FIFO_OK && count_reads == 2);
        assert(out.data.accel[0] == -16 && out.data.accel[1] == -1 && out.data.accel[2] == INT16_MAX);
        assert(out.data.gyro[0] == -4 && out.data.gyro[1] == 0 && out.data.gyro[2] == INT16_MAX);
        assert(out.data.temp == -12345 && out.timestamp_raw == 65500 && out.header == 0x78);
        push(b,h,12345,1214,0x78);
        assert(ICM45686_FifoRead(&out) == ICM45686_FIFO_OK);
        assert(out.data.accel[0] == INT16_MIN && out.data.accel[1] == 0 && out.data.accel[2] == 50);
        assert(out.data.gyro[0] == INT16_MIN && out.data.gyro[1] == 0 && out.data.gyro[2] == 10);
        assert(out.data.temp == 12345 && out.timestamp_raw == 1214);
        assert((uint16_t)(1214U - 65500U) == 1250U);
        force_count_first_zero = 0;
    }
    assert(fifo_bursts == 4);
    puts("PASS mandatory count double-read, both endiannesses, signed20 nibbles/floor scaling/saturation, full temp16 and raw timestamp wrap");
    const uint8_t headers[] = {0x70,0x7c,0xf8,0x79,0x7a,0x80,0x00};
    for (unsigned i=0; i<sizeof(headers); i++) { push(a,g,100,10,headers[i]); unchanged_failure(ICM45686_FIFO_INVALID_FRAME); }
    int32_t invalid[3] = {-524288,0,0}; push(invalid,g,100,10,0x78); unchanged_failure(ICM45686_FIFO_INVALID_FRAME);
    invalid[0] = -524273; push(a,invalid,100,10,0x78); unchanged_failure(ICM45686_FIFO_INVALID_FRAME);
    push(a,g,INT16_MIN,10,0x78); unchanged_failure(ICM45686_FIFO_INVALID_FRAME);
    puts("PASS absent timestamp, FSYNC/extended/ODR-change headers and invalid sensor/temp markers never mutate output");
    push(a,g,100,10,0x78); regs[0x19] = 1; count = 99; unsigned before = fifo_bursts;
    assert(ICM45686_FifoGetCount(&count) == ICM45686_FIFO_OVERFLOW && count == 0 && fifo_bursts == before);
    assert(ICM45686_FifoFlush() == ICM45686_FIFO_OK && fifo_count == 0);
    fail_reg = 0x12; count = 99; assert(ICM45686_FifoGetCount(&count) == ICM45686_FIFO_IO_ERROR && count == 0 && cs == 1); fail_reg = -1;
    push(a,g,100,10,0x78); fail_reg = 0x14; partial_failure = 1; unchanged_failure(ICM45686_FIFO_IO_ERROR);
    fail_reg = -1; partial_failure = 0; assert(ICM45686_FifoFlush() == ICM45686_FIFO_OK);
    flush_auto = 0; uint32_t began = ticks; assert(ICM45686_FifoFlush() == ICM45686_FIFO_TIMEOUT && ticks - began >= 1000);
    flush_auto = 1; assert(ICM45686_FifoFlush() == ICM45686_FIFO_OK);
    puts("PASS overflow is explicit, count/frame SPI errors retain output, partial read recovery flushes, flush timeout is bounded");
    regs[0x19] = 0x80; count = 99; assert(ICM45686_FifoGetCount(&count) == ICM45686_FIFO_CONFIG_ERROR && count == 0);
    assert(ICM45686_FifoGetCount(&count) == ICM45686_FIFO_CONFIG_ERROR);
    enable(0); mismatch_reg = 0x23; assert(ICM45686_FifoEnable() == ICM45686_FIFO_CONFIG_ERROR); mismatch_reg = -1;
    assert(regs[0x16] == 0 && (regs[0x21] & 1) == 0 && regs[0x1d] == 0x1e);
    assert(ICM45686_FifoGetCount(&count) == ICM45686_FIFO_CONFIG_ERROR);
    enable(0); fail_reg = 0x1d; assert(ICM45686_FifoEnable() == ICM45686_FIFO_IO_ERROR); fail_reg = -1;
    assert(ICM45686_FifoGetCount(&count) == ICM45686_FIFO_CONFIG_ERROR);
    assert(ICM45686_SourceReadRegisters(0, scratch, 21) == HAL_ERROR);
    assert(ICM45686_SourceReadMreg(0x3000, scratch) == HAL_ERROR);
    assert(ICM45686_SourceWriteMreg(0x9000,0) == HAL_ERROR);
    assert(ICM45686_SourceReadMreg(0xb000, scratch) == HAL_ERROR);
    puts("PASS hardware reset invalidates configuration, readback/SPI failures fail closed and MREG forbidden ranges are blocked");
    puts("5/5 PASS (actual driver + ASan/UBSan)"); return 0;
}
'''


def main():
    compiler = shutil.which('cc')
    if not compiler:
        raise SystemExit('A host C compiler is required')
    with tempfile.TemporaryDirectory(prefix='openimu-fifo-unit-') as temporary:
        work = Path(temporary)
        (work / 'main.h').write_text(HAL_STUB)
        for name in ('icm45686.h', 'icm45686_fifo.h', 'imu_config.h'):
            (work / name).write_bytes((ROOT / 'Core/Inc' / name).read_bytes())
        for name in ('icm45686.c', 'icm45686_fifo.c'):
            (work / name).write_bytes((ROOT / 'Core/Src' / name).read_bytes())
        (work / 'test.c').write_text(HARNESS)
        executable = work / 'test'
        subprocess.run([compiler, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                        '-DIMU_ENABLE_SOURCE_TIMESTAMP=1', '-fsanitize=address,undefined',
                        '-fno-omit-frame-pointer', '-I', str(work), str(work / 'icm45686.c'),
                        str(work / 'icm45686_fifo.c'), str(work / 'test.c'), '-o', str(executable)], check=True)
        environment = {**os.environ, 'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1',
                       'UBSAN_OPTIONS': 'halt_on_error=1'}
        subprocess.run([str(executable)], env=environment, check=True)


if __name__ == '__main__':
    main()
