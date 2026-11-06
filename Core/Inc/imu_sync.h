#ifndef IMU_SYNC_H
#define IMU_SYNC_H
#include <stdbool.h>
#include <stdint.h>
#ifndef OPENIMU_CAN_SYNC
#define OPENIMU_CAN_SYNC 0
#endif
#ifndef OPENIMU_NODE_ID
#define OPENIMU_NODE_ID 1
#endif
#if OPENIMU_NODE_ID < 1 || OPENIMU_NODE_ID > 5
#error "OPENIMU_NODE_ID must be 1..5"
#endif
#if OPENIMU_CAN_SYNC
#include "main.h"
bool IMU_SyncInit(FDCAN_HandleTypeDef *can);
void IMU_SyncTimerIRQ(void);
void IMU_SyncPoll(void);
uint32_t IMU_SyncCaptureToken(void);
void IMU_SyncSourceInvalid(void);
void IMU_SyncSourceSample(uint16_t stamp);
void IMU_SyncReport(uint32_t token, bool clean);
bool IMU_SyncExtend(uint8_t *data);
#endif
#endif
