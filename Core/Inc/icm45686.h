/**
  ******************************************************************************
  * @file    icm45686.h
  * @brief   ICM-45686 6-axis IMU (gyro + accel) driver over SPI.
  *
  *          This driver is written for the STM32G431 + HAL environment and
  *          assumes the SPI peripheral (SPI1) has been initialised with:
  *            - 8-bit data size
  *            - Mode 0 (CPOL = Low, CPHA = 1Edge)
  *            - MSB first
  *            - Hardware NSS (CS) with NSS pulse management DISABLED
  *            - SCK <= 24 MHz (12.5 MHz recommended)
  *
  *          All register definitions below are the "little-endian" register
  *          map (inv_imu_regmap_le.h from the TDK InvenSense reference driver),
  *          which matches the device power-up default (SREG_CTRL data
  *          endianness = little endian). Sensor data is therefore decoded as
  *          value = (byte1 << 8) | byte0.
  ******************************************************************************
  */
#ifndef __ICM45686_H
#define __ICM45686_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ------------------------------------------------------------------------- */
/* Register addresses (DREG_BANK1 - directly accessible over SPI)            */
/* ------------------------------------------------------------------------- */
#define ICM45686_REG_ACCEL_DATA_X1     0x00U   /* accel X high byte   */
#define ICM45686_REG_ACCEL_DATA_X0     0x01U   /* accel X low byte    */
#define ICM45686_REG_ACCEL_DATA_Y1     0x02U
#define ICM45686_REG_ACCEL_DATA_Y0     0x03U
#define ICM45686_REG_ACCEL_DATA_Z1     0x04U
#define ICM45686_REG_ACCEL_DATA_Z0     0x05U
#define ICM45686_REG_GYRO_DATA_X1      0x06U
#define ICM45686_REG_GYRO_DATA_X0      0x07U
#define ICM45686_REG_GYRO_DATA_Y1      0x08U
#define ICM45686_REG_GYRO_DATA_Y0      0x09U
#define ICM45686_REG_GYRO_DATA_Z1      0x0AU
#define ICM45686_REG_GYRO_DATA_Z0      0x0BU
#define ICM45686_REG_TEMP_DATA1        0x0CU
#define ICM45686_REG_TEMP_DATA0        0x0DU

#define ICM45686_REG_PWR_MGMT0         0x10U
#define ICM45686_REG_INT1_CONFIG0      0x16U
#define ICM45686_REG_INT1_CONFIG1      0x17U
#define ICM45686_REG_INT1_CONFIG2      0x18U
#define ICM45686_REG_INT1_STATUS0      0x19U
#define ICM45686_REG_ACCEL_CONFIG0     0x1BU
#define ICM45686_REG_GYRO_CONFIG0      0x1CU

#define ICM45686_REG_WHO_AM_I          0x72U
#define ICM45686_REG_REG_MISC2         0x7FU

/* Indirect (MREG) access registers - only needed for advanced configuration */
#define ICM45686_REG_IREG_ADDR_15_8    0x7CU
#define ICM45686_REG_IREG_ADDR_7_0     0x7DU
#define ICM45686_REG_IREG_DATA         0x7EU

/* MREG address for the sensor data endianness control register */
#define ICM45686_MREG_SREG_CTRL        0xA267U
#define ICM45686_SREG_DATA_BIG_ENDIAN  0x02U   /* sreg_data_endian_sel = 1 */

/* MREG addresses for low-noise mode configuration (match TDK reference driver) */
#define ICM45686_MREG_ACCEL_UI_LPFBW   0xA583U  /* IPREG_SYS2_REG_131: accel_ui_lpfbw_sel[2:0] */
#define ICM45686_MREG_GYRO_UI_LPFBW    0xA4ACU  /* IPREG_SYS1_REG_172: gyro_ui_lpfbw_sel[2:0]  */

/* UI low-pass filter bandwidth (divisor of the output data rate) */
#define ICM45686_UI_LPFBW_DIV_4        0x01U

/* ------------------------------------------------------------------------- */
/* WHO_AM_I                                                                   */
/* ------------------------------------------------------------------------- */
#define ICM45686_WHO_AM_I_VALUE        0xE9U

/* ------------------------------------------------------------------------- */
/* PWR_MGMT0 (0x10) - accel_mode[1:0], gyro_mode[3:2]                        */
/* ------------------------------------------------------------------------- */
#define ICM45686_PWR_ACCEL_MODE_LN     0x03U   /* low-noise mode */
#define ICM45686_PWR_GYRO_MODE_LN      0x03U   /* low-noise mode */

/* ------------------------------------------------------------------------- */
/* ACCEL_CONFIG0 (0x1B) - odr[3:0], fs_sel[6:4]                              */
/* ------------------------------------------------------------------------- */
#define ICM45686_ACCEL_ODR_800_HZ      0x06U
#define ICM45686_ACCEL_FS_2G           0x04U
#define ICM45686_ACCEL_FS_4G           0x03U
#define ICM45686_ACCEL_FS_8G           0x02U
#define ICM45686_ACCEL_FS_16G          0x01U
#define ICM45686_ACCEL_FS_32G          0x00U

/* ------------------------------------------------------------------------- */
/* GYRO_CONFIG0 (0x1C) - odr[3:0], fs_sel[7:4]                               */
/* ------------------------------------------------------------------------- */
#define ICM45686_GYRO_ODR_800_HZ       0x06U
#define ICM45686_GYRO_FS_2000_DPS      0x01U
#define ICM45686_GYRO_FS_1000_DPS      0x02U
#define ICM45686_GYRO_FS_500_DPS       0x03U
#define ICM45686_GYRO_FS_250_DPS       0x04U
#define ICM45686_GYRO_FS_4000_DPS      0x00U

/* ------------------------------------------------------------------------- */
/* INT1_CONFIG0 (0x16)                                                       */
/* ------------------------------------------------------------------------- */
#define ICM45686_INT1_EN_UI_DRDY       0x04U   /* int1_status_en_drdy (bit 2) */

/* ------------------------------------------------------------------------- */
/* INT1_CONFIG2 (0x18) - polarity[0], mode[1], drive[2]                      */
/* ------------------------------------------------------------------------- */
#define ICM45686_INT1_POLARITY_ACTIVE_HIGH  0x01U
#define ICM45686_INT1_POLARITY_ACTIVE_LOW   0x00U

/* ------------------------------------------------------------------------- */
/* REG_MISC2 (0x7F) - soft_rst (bit 1)                                      */
/* ------------------------------------------------------------------------- */
#define ICM45686_SOFT_RESET            0x02U

/* ------------------------------------------------------------------------- */
/* Configurable full-scale ranges (sensitivity in LSB per unit)             */
/* ------------------------------------------------------------------------- */
#define ICM45686_ACCEL_FS_CFG         ICM45686_ACCEL_FS_4G     /* +/-4 g    */
#define ICM45686_GYRO_FS_CFG          ICM45686_GYRO_FS_2000_DPS/* +/-2000 dps */

#define ICM45686_ACCEL_SENS_LSB_PER_G   8192.0f   /* +/-4 g   -> 8192 LSB/g  */
#define ICM45686_GYRO_SENS_LSB_PER_DPS  16.4f     /* +/-2000  -> 16.4 LSB/dps*/

/* Raw sensor data structure (all values are 16-bit signed, little-endian) */
typedef struct {
    int16_t accel[3];   /* X, Y, Z raw accel  */
    int16_t gyro[3];    /* X, Y, Z raw gyro   */
    int16_t temp;       /* raw temperature    */
} ICM45686_Data_t;

/* Public API ----------------------------------------------------------------- */
HAL_StatusTypeDef ICM45686_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ICM45686_ReadData(ICM45686_Data_t *data);
HAL_StatusTypeDef ICM45686_ReadRegister(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ICM45686_WriteRegister(uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* __ICM45686_H */
