#ifndef IMU_TIMESTAMP_H
#define IMU_TIMESTAMP_H

#include "imu_config.h"
#if IMU_ENABLE_SOURCE_TIMESTAMP
#include <stdbool.h>
#include <stdint.h>

/* Sensor-local time since the first valid FIFO sample of the current epoch.
 * Not STM32 uptime or UTC. Hardware FIFO timestamp resolution must be 1us.
 * The caller must reset this state on every FIFO continuity loss. */
typedef struct {
    uint64_t time_us;
    uint16_t previous_raw;
    bool initialized;
} IMU_Timestamp_t;

void IMU_TimestampReset(IMU_Timestamp_t *clock);
/* Zero or implausibly large increments are rejected without changing state.
 * Correct unfolding requires consecutive, valid fixed-800Hz FIFO samples;
 * this function cannot infer multiple missing 16-bit wraps after data loss. */
bool IMU_TimestampAdvance(IMU_Timestamp_t *clock, uint16_t raw,
                          uint64_t *source_time_us);
#endif
#endif
