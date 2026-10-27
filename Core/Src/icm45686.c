/**
  ******************************************************************************
  * @file    icm45686.c
  * @brief   ICM-45686 6-axis IMU driver implementation (SPI, blocking mode).
  *
  *          SPI protocol (InvenSense convention):
  *            - Write: first byte = (reg & 0x7F), followed by data bytes.
  *            - Read : first byte = (reg | 0x80), followed by dummy bytes; the
  *                     device returns data on MISO while the dummy bytes are
  *                     clocked out.
  *            - Burst read/write uses the register auto-increment feature and a
  *              single HAL_SPI_TransmitReceive call. Chip-select (PA4) is driven
  *              by software around each transaction.
  *
  *          The register map and default (little-endian) data format follow
  *          the TDK InvenSense reference driver (inv_imu_regmap_le.h).
  ******************************************************************************
  */
#include "icm45686.h"

/* Software chip-select (CS) on PA4 - toggled manually around each transaction.
 * This sidesteps any STM32 hardware-NSS timing quirks. */
#define ICM45686_CS_PORT    GPIOA
#define ICM45686_CS_PIN     GPIO_PIN_4
#define ICM45686_CS_LOW()   HAL_GPIO_WritePin(ICM45686_CS_PORT, ICM45686_CS_PIN, GPIO_PIN_RESET)
#define ICM45686_CS_HIGH()  HAL_GPIO_WritePin(ICM45686_CS_PORT, ICM45686_CS_PIN, GPIO_PIN_SET)

/* ------------------------------------------------------------------------- */
/* Private variables                                                         */
/* ------------------------------------------------------------------------- */
static SPI_HandleTypeDef *icm_spi = NULL;
static uint8_t icm_endian_big = 0U;   /* 0 = little-endian, 1 = big-endian */

/* ------------------------------------------------------------------------- */
/* SPI low-level transport                                                   */
/* ------------------------------------------------------------------------- */

/**
  * @brief  Write one or more bytes to consecutive registers.
  * @param  reg   Starting register address.
  * @param  buf   Data buffer.
  * @param  len   Number of bytes to write.
  */
static HAL_StatusTypeDef ICM45686_WriteRegisters(uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint8_t tx[17]; /* 1 address byte + up to 16 data bytes */
    uint8_t rx[17];

    if (icm_spi == NULL || buf == NULL || len == 0U || len > (sizeof(tx) - 1U)) {
        return HAL_ERROR;
    }

    tx[0] = reg & 0x7FU;
    for (uint16_t i = 0; i < len; i++) {
        tx[i + 1U] = buf[i];
    }

    /* Full-duplex transfer (MISO is ignored). CS asserted for the transaction. */
    ICM45686_CS_LOW();
    if (HAL_SPI_TransmitReceive(icm_spi, tx, rx, (uint16_t)(len + 1U), HAL_MAX_DELAY) != HAL_OK) {
        ICM45686_CS_HIGH();
        return HAL_ERROR;
    }
    ICM45686_CS_HIGH();
    return HAL_OK;
}

/**
  * @brief  Read one or more bytes from consecutive registers.
  * @param  reg   Starting register address.
  * @param  buf   Output buffer.
  * @param  len   Number of bytes to read.
  */
static HAL_StatusTypeDef ICM45686_ReadRegisters(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t tx[17]; /* 1 address byte + up to 16 data bytes */
    uint8_t rx[17];

    if (icm_spi == NULL || buf == NULL || len == 0U || len > (sizeof(tx) - 1U)) {
        return HAL_ERROR;
    }

    tx[0] = reg | 0x80U; /* read bit */
    for (uint16_t i = 1; i <= len; i++) {
        tx[i] = 0x00U;    /* dummy bytes clocked out to shift data in */
    }

    ICM45686_CS_LOW();
    if (HAL_SPI_TransmitReceive(icm_spi, tx, rx, (uint16_t)(len + 1U), HAL_MAX_DELAY) != HAL_OK) {
        ICM45686_CS_HIGH();
        return HAL_ERROR;
    }
    ICM45686_CS_HIGH();

    for (uint16_t i = 0; i < len; i++) {
        buf[i] = rx[i + 1U];
    }
    return HAL_OK;
}

/**
  * @brief  Read one or more bytes from an indirect (MREG) register.
  * @param  addr  16-bit MREG address.
  * @param  buf   Output buffer.
  * @param  len   Number of bytes to read.
  */
static HAL_StatusTypeDef ICM45686_ReadMreg(uint16_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t a[2];

    a[0] = (uint8_t)(addr >> 8);
    a[1] = (uint8_t)(addr & 0xFFU);

    /* Write the 16-bit MREG address... */
    if (ICM45686_WriteRegisters(ICM45686_REG_IREG_ADDR_15_8, a, 2U) != HAL_OK) {
        return HAL_ERROR;
    }
    /* ...then read back data one byte at a time (device auto-increments). */
    for (uint16_t i = 0; i < len; i++) {
        if (ICM45686_ReadRegister(ICM45686_REG_IREG_DATA, &buf[i]) != HAL_OK) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
  * @brief  Write one byte to an indirect (MREG) register.
  * @param  addr   16-bit MREG address.
  * @param  value  Value to write.
  */
static HAL_StatusTypeDef ICM45686_WriteMreg(uint16_t addr, uint8_t value)
{
    uint8_t a[3];

    a[0] = (uint8_t)(addr >> 8);
    a[1] = (uint8_t)(addr & 0xFFU);
    a[2] = value;

    /* Burst-write the 16-bit MREG address plus the data byte. */
    return ICM45686_WriteRegisters(ICM45686_REG_IREG_ADDR_15_8, a, 3U);
}

/**
  * @brief  Set the 3-bit UI low-pass-filter bandwidth field of an MREG register
  *         with a read-modify-write, leaving the other bits untouched.
  * @note   Mirrors TDK's inv_imu_set_accel_ln_bw() / inv_imu_set_gyro_ln_bw()
  *         and the BeijingDynamics open_sprite_runtime configuration.
  * @param  addr  16-bit MREG address (IPREG_SYS2_REG_131 / IPREG_SYS1_REG_172).
  * @param  value LPFBW selection (0 = no filter, 1 = ODR/4 ... 6 = ODR/128).
  */
static HAL_StatusTypeDef ICM45686_SetLpfBw(uint16_t addr, uint8_t value)
{
    uint8_t v = 0U;

    if (ICM45686_ReadMreg(addr, &v, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    v = (uint8_t)((v & 0xF8U) | (value & 0x07U));
    return ICM45686_WriteMreg(addr, v);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

HAL_StatusTypeDef ICM45686_WriteRegister(uint8_t reg, uint8_t value)
{
    return ICM45686_WriteRegisters(reg, &value, 1U);
}

HAL_StatusTypeDef ICM45686_ReadRegister(uint8_t reg, uint8_t *value)
{
    return ICM45686_ReadRegisters(reg, value, 1U);
}

/**
  * @brief  Write a register and verify it by reading it back.
  * @note   Returns HAL_ERROR when the write cannot be verified (SPI write path
  *         broken, or the register is not writable).
  */
static HAL_StatusTypeDef ICM45686_WriteRegisterChecked(uint8_t reg, uint8_t value)
{
    uint8_t rb = 0U;
    if (ICM45686_WriteRegister(reg, value) != HAL_OK) {
        return HAL_ERROR;
    }
    if (ICM45686_ReadRegister(reg, &rb) != HAL_OK) {
        return HAL_ERROR;
    }
    if (rb != value) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @brief  Initialise the ICM-45686.
  * @note   Must be called after HAL_SPI_Init() and before the FreeRTOS
  *         scheduler reads data. Blocks using HAL_Delay() for the required
  *         reset and start-up settling.
  */
HAL_StatusTypeDef ICM45686_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t who_am_i = 0U;
    uint8_t value = 0U;

    if (hspi == NULL) {
        return HAL_ERROR;
    }
    icm_spi = hspi;

    /* 1. Soft reset */
    if (ICM45686_WriteRegister(ICM45686_REG_REG_MISC2, ICM45686_SOFT_RESET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10); /* wait for reset to complete */

    /* 2. Check WHO_AM_I */
    if (ICM45686_ReadRegister(ICM45686_REG_WHO_AM_I, &who_am_i) != HAL_OK) {
        return HAL_ERROR;
    }
    if (who_am_i != ICM45686_WHO_AM_I_VALUE) {
        return HAL_ERROR;
    }

    /* 2b. Read the sensor data endianness (defaults to little-endian). */
    if (ICM45686_ReadMreg(ICM45686_MREG_SREG_CTRL, &value, 1U) != HAL_OK) {
        icm_endian_big = 0U; /* fall back to the power-up default */
    } else {
        icm_endian_big = ((value & ICM45686_SREG_DATA_BIG_ENDIAN) != 0U) ? 1U : 0U;
    }

    /* 3. Clear interrupt routing first; the UI data-ready source is enabled
     *    AFTER power-on (see step 10), matching the working BeijingDynamics /
     *    open_sprite_runtime configuration. */
    if (ICM45686_WriteRegister(ICM45686_REG_INT1_CONFIG0, 0x00U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (ICM45686_WriteRegister(ICM45686_REG_INT1_CONFIG1, 0x00U) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 4. INT1 pin config: active-high, pulse, push-pull (write + readback) */
    if (ICM45686_WriteRegisterChecked(ICM45686_REG_INT1_CONFIG2, ICM45686_INT1_POLARITY_ACTIVE_HIGH) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 5. Configure accel ODR = 800 Hz and full-scale range (write + readback) */
    value = (uint8_t)((ICM45686_ACCEL_FS_CFG << 4) | ICM45686_ACCEL_ODR_800_HZ);
    if (ICM45686_WriteRegisterChecked(ICM45686_REG_ACCEL_CONFIG0, value) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 6. Configure gyro ODR = 800 Hz and full-scale range (write + readback) */
    value = (uint8_t)((ICM45686_GYRO_FS_CFG << 4) | ICM45686_GYRO_ODR_800_HZ);
    if (ICM45686_WriteRegisterChecked(ICM45686_REG_GYRO_CONFIG0, value) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 7. UI low-pass filter bandwidth = ODR/4 (low-noise mode). Same value as
     *    the BeijingDynamics open_sprite_runtime reference; read-modify-write
     *    keeps the other bits of the MREG intact. */
    if (ICM45686_SetLpfBw(ICM45686_MREG_ACCEL_UI_LPFBW, ICM45686_UI_LPFBW_DIV_4) != HAL_OK) {
        return HAL_ERROR;
    }
    if (ICM45686_SetLpfBw(ICM45686_MREG_GYRO_UI_LPFBW, ICM45686_UI_LPFBW_DIV_4) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 8. Power on accel + gyro in low-noise mode (write + readback) */
    value = (uint8_t)((ICM45686_PWR_GYRO_MODE_LN << 2) | ICM45686_PWR_ACCEL_MODE_LN);
    if (ICM45686_WriteRegisterChecked(ICM45686_REG_PWR_MGMT0, value) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 9. Wait for sensor start-up (gyro ~70 ms) before enabling the DRDY
     *    interrupt routing. */
    HAL_Delay(100);

    /* 10. Route UI data-ready to INT1 (enabled AFTER power-on; write + readback) */
    if (ICM45686_WriteRegisterChecked(ICM45686_REG_INT1_CONFIG0, ICM45686_INT1_EN_UI_DRDY) != HAL_OK) {
        return HAL_ERROR;
    }
    if (ICM45686_WriteRegister(ICM45686_REG_INT1_CONFIG1, 0x00U) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Combine two bytes into a 16-bit signed value according to the
  *         device's current data endianness.
  */
static inline int16_t ICM45686_Combine(const uint8_t *p)
{
    if (icm_endian_big != 0U) {
        return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
    }
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/**
  * @brief  Read the latest accel + gyro + temperature sample.
  * @note   Performs a single 14-byte burst read from ACCEL_DATA_X1.
  */
HAL_StatusTypeDef ICM45686_ReadData(ICM45686_Data_t *data)
{
    uint8_t buf[14];

    if (data == NULL) {
        return HAL_ERROR;
    }

    if (ICM45686_ReadRegisters(ICM45686_REG_ACCEL_DATA_X1, buf, sizeof(buf)) != HAL_OK) {
        return HAL_ERROR;
    }

    data->accel[0] = ICM45686_Combine(&buf[0]);
    data->accel[1] = ICM45686_Combine(&buf[2]);
    data->accel[2] = ICM45686_Combine(&buf[4]);
    data->gyro[0]  = ICM45686_Combine(&buf[6]);
    data->gyro[1]  = ICM45686_Combine(&buf[8]);
    data->gyro[2]  = ICM45686_Combine(&buf[10]);
    data->temp      = ICM45686_Combine(&buf[12]);

    return HAL_OK;
}
