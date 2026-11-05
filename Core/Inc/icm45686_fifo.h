#ifndef ICM45686_FIFO_H
#define ICM45686_FIFO_H

#include "imu_config.h"
#if IMU_ENABLE_SOURCE_TIMESTAMP
#include "icm45686.h"

/* Hardware FIFO20, cumulative 1-us timestamp, no FSYNC or delta semantics.
 * All APIs are task-context, single-owner: serialize with every other IMU SPI
 * operation. Off builds contain none of this optional driver. */
typedef enum {
    ICM45686_FIFO_OK = 0,
    ICM45686_FIFO_EMPTY,
    ICM45686_FIFO_OVERFLOW,
    ICM45686_FIFO_INVALID_FRAME,
    ICM45686_FIFO_IO_ERROR,
    ICM45686_FIFO_CONFIG_ERROR,
    ICM45686_FIFO_TIMEOUT,
} ICM45686_FifoResult_t;

typedef struct {
    ICM45686_Data_t data;
    uint16_t timestamp_raw;
    uint8_t header;
} ICM45686_FifoSample_t;

/* Call after the unchanged ICM45686_Init. Keeps its ODR, filter and configured
 * physical output units; FIFO20 fixed-range values are scaled/saturated back to
 * those 16-bit units. Uses snapshot mode, watermark=1 complete packet. */
ICM45686_FifoResult_t ICM45686_FifoEnable(void);
/* Count is complete packets, not bytes. Performs the AN-000364 double read.
 * On any failure *frames is zero. Overflow/reset/config errors require recovery
 * and a new timestamp epoch; never integrate across that discontinuity. */
ICM45686_FifoResult_t ICM45686_FifoGetCount(uint16_t *frames);
/* Safe read-one: status/count check followed by exactly one complete FIFO20
 * burst. Output is committed only on success; no automatic flush/error hiding. */
ICM45686_FifoResult_t ICM45686_FifoRead(ICM45686_FifoSample_t *sample);
/* Drops buffered data. Caller must explicitly begin a new timestamp epoch;
 * this does not reset the sensor's cumulative timestamp counter. */
ICM45686_FifoResult_t ICM45686_FifoFlush(void);

#endif /* IMU_ENABLE_SOURCE_TIMESTAMP */
#endif
