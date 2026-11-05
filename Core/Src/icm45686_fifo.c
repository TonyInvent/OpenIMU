#include "icm45686_fifo.h"
#if IMU_ENABLE_SOURCE_TIMESTAMP
#include <limits.h>
#include <stddef.h>

/* Register fields and FIFO20 extraction are based on the official TDK driver:
 * https://github.com/tdk-invn-oss/motion.mcu.icm45686.driver
 * commit 0a5bed6975b5cdeecb98f95db420f3058dd88547
 * icm45686/imu/{inv_imu_regmap_le.h,inv_imu_defs.h,inv_imu_driver.c},
 * examples/basic_read_fifo/basic_read_fifo.c. No guessed ICM-426xx register map.
 * Snapshot mode follows the basic example and avoids the AN-000364 streaming
 * rule requiring a host to leave the newest (Mth) frame unread. */
#define REG_FIFO_COUNT_0      0x12U
#define REG_FIFO_DATA         0x14U
#define REG_FIFO_CONFIG0      0x1DU
#define REG_FIFO_CONFIG2      0x20U
#define REG_FIFO_CONFIG3      0x21U
#define REG_TMST_WOM_CONFIG   0x23U
#define REG_FSYNC_CONFIG0     0x24U
#define REG_ODR_DECIMATE      0x28U
#define MREG_SMC_CONTROL_0    0xA258U
#define FIFO_DEPTH_MAX        0x1EU
#define FIFO_MODE_SNAPSHOT    0x80U
#define FIFO_SENSORS_HIRES    0x0EU
#define FIFO_INTERFACE_ENABLE 0x01U
#define FIFO_TIMESTAMP        0x02U
#define FIFO_WATERMARK_GE     0x08U
#define FIFO_FLUSH            0x80U
#define FIFO_STATUS_FULL      0x01U
#define FIFO_STATUS_RESET     0x80U
#define FIFO20_HEADER         0x78U
#define FIFO20_BYTES          20U

#if ICM45686_ACCEL_FS_CFG > 4 || ICM45686_GYRO_FS_CFG > 4
#error "FIFO20 scaling supports the original full-scale encodings 0..4 only"
#endif

static uint8_t s_enabled;
static uint8_t s_big_endian;

static ICM45686_FifoResult_t read_register(uint8_t reg, uint8_t *value)
{
    return ICM45686_SourceReadRegisters(reg, value, 1U) == HAL_OK ?
        ICM45686_FIFO_OK : ICM45686_FIFO_IO_ERROR;
}

static ICM45686_FifoResult_t write_checked(uint8_t reg, uint8_t value, uint8_t mask)
{
    uint8_t readback;
    if (ICM45686_SourceWriteRegisters(reg, &value, 1U) != HAL_OK ||
        ICM45686_SourceReadRegisters(reg, &readback, 1U) != HAL_OK)
        return ICM45686_FIFO_IO_ERROR;
    return (readback & mask) == (value & mask) ? ICM45686_FIFO_OK : ICM45686_FIFO_CONFIG_ERROR;
}

static ICM45686_FifoResult_t mreg_update(uint16_t address, uint8_t clear, uint8_t set)
{
    uint8_t value, check;
    if (ICM45686_SourceReadMreg(address, &value) != HAL_OK) return ICM45686_FIFO_IO_ERROR;
    value = (uint8_t)((value & (uint8_t)~clear) | set);
    if (ICM45686_SourceWriteMreg(address, value) != HAL_OK ||
        ICM45686_SourceReadMreg(address, &check) != HAL_OK) return ICM45686_FIFO_IO_ERROR;
    return check == value ? ICM45686_FIFO_OK : ICM45686_FIFO_CONFIG_ERROR;
}

#define REQUIRE(call) do { ICM45686_FifoResult_t result_ = (call); \
    if (result_ != ICM45686_FIFO_OK) return result_; } while (0)

static uint16_t combine_u16(const uint8_t *bytes)
{
    return s_big_endian ? (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]) :
        (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
}

ICM45686_FifoResult_t ICM45686_FifoFlush(void)
{
    uint8_t value;
    REQUIRE(read_register(REG_FIFO_CONFIG2, &value));
    value |= FIFO_FLUSH;
    if (ICM45686_SourceWriteRegisters(REG_FIFO_CONFIG2, &value, 1U) != HAL_OK)
        return ICM45686_FIFO_IO_ERROR;
    uint32_t began = HAL_GetTick();
    /* Official driver waits 10 us, then polls up to one second. A conservative
     * 1-ms delay is acceptable for explicit error recovery, never per sample. */
    do {
        HAL_Delay(1U);
        REQUIRE(read_register(REG_FIFO_CONFIG2, &value));
        if ((value & FIFO_FLUSH) == 0U) return ICM45686_FIFO_OK;
    } while ((uint32_t)(HAL_GetTick() - began) < 1000U);
    return ICM45686_FIFO_TIMEOUT;
}

static ICM45686_FifoResult_t configure_fifo(void)
{
    uint8_t config[6], value, interrupt;
    /* Ensure the original 800-Hz/full-scale configuration really exists. Do not
     * silently turn a reset/default sensor into seemingly valid timestamp data. */
    REQUIRE(read_register(ICM45686_REG_ACCEL_CONFIG0, &value));
    if ((value & 0x7FU) != (uint8_t)((ICM45686_ACCEL_FS_CFG << 4) | ICM45686_ACCEL_ODR_800_HZ))
        return ICM45686_FIFO_CONFIG_ERROR;
    REQUIRE(read_register(ICM45686_REG_GYRO_CONFIG0, &value));
    if (value != (uint8_t)((ICM45686_GYRO_FS_CFG << 4) | ICM45686_GYRO_ODR_800_HZ))
        return ICM45686_FIFO_CONFIG_ERROR;
    if (ICM45686_SourceReadMreg(ICM45686_MREG_SREG_CTRL, &value) != HAL_OK)
        return ICM45686_FIFO_IO_ERROR;
    s_big_endian = (value & ICM45686_SREG_DATA_BIG_ENDIAN) != 0U ? 1U : 0U;
    REQUIRE(read_register(ICM45686_REG_INT1_CONFIG0, &interrupt));
    REQUIRE(write_checked(ICM45686_REG_INT1_CONFIG0, (uint8_t)(interrupt & 0xF8U), 0xFFU));
    if (ICM45686_SourceReadRegisters(REG_FIFO_CONFIG0, config, sizeof(config)) != HAL_OK)
        return ICM45686_FIFO_IO_ERROR;
    REQUIRE(write_checked(REG_FIFO_CONFIG3, (uint8_t)(config[4] & (uint8_t)~FIFO_INTERFACE_ENABLE), 0x3FU));
    REQUIRE(write_checked(REG_FIFO_CONFIG0, (uint8_t)(config[0] & 0x3FU), 0xFFU));
    config[0] = FIFO_DEPTH_MAX;
    config[1] = 1U; /* watermark is packets, not bytes; write low then high */
    config[2] = 0U;
    config[3] = (uint8_t)((config[3] & 0x77U) | FIFO_WATERMARK_GE);
    config[4] = (uint8_t)((config[4] & 0xC0U) | FIFO_SENSORS_HIRES);
    config[5] = (uint8_t)((config[5] & 0xC0U) | FIFO_TIMESTAMP); /* no compression/external sensors */
    if (ICM45686_SourceWriteRegisters(REG_FIFO_CONFIG0, config, sizeof(config)) != HAL_OK)
        return ICM45686_FIFO_IO_ERROR;
    uint8_t check[6];
    if (ICM45686_SourceReadRegisters(REG_FIFO_CONFIG0, check, sizeof(check)) != HAL_OK)
        return ICM45686_FIFO_IO_ERROR;
    for (unsigned i = 0U; i < sizeof(config); i++)
        if (check[i] != config[i]) return ICM45686_FIFO_CONFIG_ERROR;
    /* TMST_RESOL bit5=0 selects 1 us; bit6=0 disables delta timestamps. */
    REQUIRE(read_register(REG_TMST_WOM_CONFIG, &value));
    REQUIRE(write_checked(REG_TMST_WOM_CONFIG, (uint8_t)(value & 0x9FU), 0x7FU));
    /* FSYNC tag selection disabled; SMC timestamp on but FSYNC conversion off. */
    REQUIRE(read_register(REG_FSYNC_CONFIG0, &value));
    REQUIRE(write_checked(REG_FSYNC_CONFIG0, (uint8_t)(value & 0xF8U), 0x0FU));
    REQUIRE(mreg_update(MREG_SMC_CONTROL_0, 0x03U, 0x01U));
    /* Equal accel/gyro FIFO rates, no decimation. Original ODR/filter/power
     * registers and their original configurable ranges remain unchanged. */
    REQUIRE(write_checked(REG_ODR_DECIMATE, 0x00U, 0xFFU));
    REQUIRE(ICM45686_FifoFlush());
    REQUIRE(read_register(ICM45686_REG_INT1_STATUS0, &value)); /* clear old status */
    REQUIRE(write_checked(REG_FIFO_CONFIG0, FIFO_MODE_SNAPSHOT | FIFO_DEPTH_MAX, 0xFFU));
    REQUIRE(write_checked(REG_FIFO_CONFIG3, (uint8_t)(config[4] | FIFO_INTERFACE_ENABLE), 0x3FU));
    /* Route FIFO threshold/full instead of UI DRDY; keep the original INT1
     * polarity, pulse mode and drive configuration untouched. */
    REQUIRE(write_checked(ICM45686_REG_INT1_CONFIG0, (uint8_t)((interrupt & 0xF8U) | 0x03U), 0xFFU));
    return ICM45686_FIFO_OK;
}

ICM45686_FifoResult_t ICM45686_FifoEnable(void)
{
    s_enabled = 0U;
    ICM45686_FifoResult_t result = configure_fifo();
    if (result == ICM45686_FIFO_OK) s_enabled = 1U;
    else {
        /* Best-effort fail-closed cleanup; do not pretend configuration worked. */
        const uint8_t off = 0U;
        const uint8_t bypass = FIFO_DEPTH_MAX;
        uint8_t interface = 0U;
        (void)ICM45686_SourceWriteRegisters(ICM45686_REG_INT1_CONFIG0, &off, 1U);
        /* Stop the interface without changing its format while still active. */
        if (ICM45686_SourceReadRegisters(REG_FIFO_CONFIG3, &interface, 1U) == HAL_OK) {
            interface &= (uint8_t)~FIFO_INTERFACE_ENABLE;
            (void)ICM45686_SourceWriteRegisters(REG_FIFO_CONFIG3, &interface, 1U);
        }
        (void)ICM45686_SourceWriteRegisters(REG_FIFO_CONFIG0, &bypass, 1U);
    }
    return result;
}

ICM45686_FifoResult_t ICM45686_FifoGetCount(uint16_t *frames)
{
    uint8_t status, count[2];
    if (frames == NULL) return ICM45686_FIFO_CONFIG_ERROR;
    *frames = 0U;
    if (s_enabled == 0U) return ICM45686_FIFO_CONFIG_ERROR;
    REQUIRE(read_register(ICM45686_REG_INT1_STATUS0, &status));
    if ((status & FIFO_STATUS_RESET) != 0U) {
        s_enabled = 0U;
        return ICM45686_FIFO_CONFIG_ERROR;
    }
    if ((status & FIFO_STATUS_FULL) != 0U) return ICM45686_FIFO_OVERFLOW;
    /* AN-000364 section 2.2: read twice and use the second packet count. */
    if (ICM45686_SourceReadRegisters(REG_FIFO_COUNT_0, count, sizeof(count)) != HAL_OK ||
        ICM45686_SourceReadRegisters(REG_FIFO_COUNT_0, count, sizeof(count)) != HAL_OK)
        return ICM45686_FIFO_IO_ERROR;
    *frames = combine_u16(count);
    return *frames == 0U ? ICM45686_FIFO_EMPTY : ICM45686_FIFO_OK;
}

static int32_t signed20(uint16_t high, uint8_t low)
{
    uint32_t raw = ((uint32_t)high << 4) | (uint32_t)(low & 0x0FU);
    return (raw & 0x80000U) != 0U ? (int32_t)raw - 0x100000 : (int32_t)raw;
}

static int16_t rescale_saturate(int32_t raw, unsigned shift)
{
    int32_t divisor = (int32_t)(1UL << shift);
    /* Explicit arithmetic-right-shift semantics without implementation-defined
     * signed shifting or undefined left-shifting of negative FIFO values. */
    int32_t scaled = raw >= 0 ? raw / divisor : -((-raw + divisor - 1) / divisor);
    if (scaled > INT16_MAX) return INT16_MAX;
    if (scaled < INT16_MIN) return INT16_MIN;
    return (int16_t)scaled;
}

ICM45686_FifoResult_t ICM45686_FifoRead(ICM45686_FifoSample_t *sample)
{
    uint16_t count;
    uint8_t frame[FIFO20_BYTES];
    if (sample == NULL) return ICM45686_FIFO_CONFIG_ERROR;
    REQUIRE(ICM45686_FifoGetCount(&count));
    if (ICM45686_SourceReadRegisters(REG_FIFO_DATA, frame, sizeof(frame)) != HAL_OK)
        return ICM45686_FIFO_IO_ERROR;
    /* Require both fresh sensors, hires and timestamp, no extended/FSYNC/ODR
     * change flags. A FSYNC duration must never masquerade as cumulative time. */
    if (frame[0] != FIFO20_HEADER) return ICM45686_FIFO_INVALID_FRAME;
    ICM45686_FifoSample_t decoded = {0};
    for (unsigned axis = 0U; axis < 3U; axis++) {
        uint16_t accel = combine_u16(frame + 1U + axis * 2U);
        uint16_t gyro = combine_u16(frame + 7U + axis * 2U);
        /* Official FIFO invalid marker is high16 == 0x8000, regardless of nibble. */
        if (accel == 0x8000U || gyro == 0x8000U) return ICM45686_FIFO_INVALID_FRAME;
        decoded.data.accel[axis] = rescale_saturate(signed20(accel, frame[17U + axis] >> 4),
                                                  4U - ICM45686_ACCEL_FS_CFG);
        decoded.data.gyro[axis] = rescale_saturate(signed20(gyro, frame[17U + axis]),
                                                 4U - ICM45686_GYRO_FS_CFG);
    }
    uint16_t temperature = combine_u16(frame + 13U);
    if (temperature == 0x8000U) return ICM45686_FIFO_INVALID_FRAME;
    decoded.data.temp = temperature <= INT16_MAX ? (int16_t)temperature :
        (int16_t)((int32_t)temperature - 0x10000);
    decoded.timestamp_raw = combine_u16(frame + 15U);
    decoded.header = frame[0];
    *sample = decoded;
    return ICM45686_FIFO_OK;
}
#undef REQUIRE
#endif /* IMU_ENABLE_SOURCE_TIMESTAMP */
