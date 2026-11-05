#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* Opt-in only. The default retains the original direct-register acquisition
 * and 16-byte CAN payload. Enable with -DIMU_ENABLE_SOURCE_TIMESTAMP=1. */
#ifndef IMU_ENABLE_SOURCE_TIMESTAMP
#define IMU_ENABLE_SOURCE_TIMESTAMP 0
#endif

#if IMU_ENABLE_SOURCE_TIMESTAMP != 0 && IMU_ENABLE_SOURCE_TIMESTAMP != 1
#error "IMU_ENABLE_SOURCE_TIMESTAMP must be 0 or 1"
#endif

#endif
