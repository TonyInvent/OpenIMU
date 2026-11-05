#include "imu_timestamp.h"
#if IMU_ENABLE_SOURCE_TIMESTAMP
#include <stddef.h>

/* Fixed 800Hz sample periods are nominally 1250us. This deliberately loose
 * sanity bound rejects invalid/reset timestamps; it is not a loss-recovery
 * algorithm. FIFO full, parse/SPI error and flush always start a new epoch. */
#define MAX_CONTINUOUS_STEP_US 10000U

void IMU_TimestampReset(IMU_Timestamp_t *clock)
{
    if (clock != NULL) {
        clock->time_us = 0U;
        clock->previous_raw = 0U;
        clock->initialized = false;
    }
}

bool IMU_TimestampAdvance(IMU_Timestamp_t *clock, uint16_t raw,
                          uint64_t *source_time_us)
{
    if (clock == NULL || source_time_us == NULL) return false;
    if (!clock->initialized) {
        clock->previous_raw = raw;
        clock->initialized = true;
        clock->time_us = 0U;
    } else {
        uint16_t delta = (uint16_t)(raw - clock->previous_raw);
        if (delta == 0U || delta > MAX_CONTINUOUS_STEP_US ||
            UINT64_MAX - clock->time_us < delta) return false;
        clock->time_us += delta;
        clock->previous_raw = raw;
    }
    *source_time_us = clock->time_us;
    return true;
}
#endif
