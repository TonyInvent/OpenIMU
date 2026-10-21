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
  *              single HAL_SPI_TransmitReceive call so that the hardware NSS
  *              (chip select) stays asserted for the whole transaction.
  *
  *          The register map and default (little-endian) data format follow
  *          the TDK InvenSense reference driver (inv_imu_regmap_le.h).
  ******************************************************************************
  */
#include "icm45686.h"

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

    if (icm_spi == NULL || buf == NULL || len == 0U || len > (sizeof(tx) - 1U)) {
        return HAL_ERROR;
    }

    tx[0] = reg & 0x7FU;
    for (uint16_t i = 0; i < len; i++) {
        tx[i + 1U] = buf[i];
    }

    /* A single transfer keeps NSS asserted for the whole transaction. */
    if (HAL_SPI_Transmit(icm_spi, tx, (uint16_t)(len + 1U), HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
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

    if (HAL_SPI_TransmitReceive(icm_spi, tx, rx, (uint16_t)(len + 1U), HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

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
    HAL_Delay(1); /* wait for reset to complete */

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

    /* 3. Power on accel + gyro in low-noise mode */
    value = (uint8_t)((ICM45686_PWR_GYRO_MODE_LN << 2) | ICM45686_PWR_ACCEL_MODE_LN);
    if (ICM45686_WriteRegister(ICM45686_REG_PWR_MGMT0, value) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 4. Configure accel ODR = 800 Hz and full-scale range */
    value = (uint8_t)((ICM45686_ACCEL_FS_CFG << 4) | ICM45686_ACCEL_ODR_800_HZ);
    if (ICM45686_WriteRegister(ICM45686_REG_ACCEL_CONFIG0, value) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 5. Configure gyro ODR = 800 Hz and full-scale range */
    value = (uint8_t)((ICM45686_GYRO_FS_CFG << 4) | ICM45686_GYRO_ODR_800_HZ);
    if (ICM45686_WriteRegister(ICM45686_REG_GYRO_CONFIG0, value) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 6. Route UI data-ready to INT1 as an active-high pulse (rising edge
     *    compatible with the EXTI0 pin on PB0). */
    if (ICM45686_WriteRegister(ICM45686_REG_INT1_CONFIG1, 0x00U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (ICM45686_WriteRegister(ICM45686_REG_INT1_CONFIG0, ICM45686_INT1_EN_UI_DRDY) != HAL_OK) {
        return HAL_ERROR;
    }
    if (ICM45686_WriteRegister(ICM45686_REG_INT1_CONFIG2, ICM45686_INT1_POLARITY_ACTIVE_HIGH) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Gyro start-up time is ~70 ms; let the first data-ready interrupt gate
     * the read loop, so no explicit settling delay is required here. */

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
