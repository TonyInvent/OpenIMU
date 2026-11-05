/**
  ******************************************************************************
  * @file    imu_task.h
  * @brief   FreeRTOS task that reads ICM-45686 samples on data-ready and sends
  *          them over CAN FD (ID 0x01) at 800 Hz.
  ******************************************************************************
  */
#ifndef __IMU_TASK_H
#define __IMU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "imu_config.h"

/**
  * @brief  CAN FD frame identifier used for IMU data.
  */
#define IMU_CAN_ID              0x01U

/**
  * @brief  Payload size: original 16 bytes, or optional timestamp format 32.
  */
#if IMU_ENABLE_SOURCE_TIMESTAMP
#define IMU_CAN_DLC_BYTES       32U
#define IMU_TIMESTAMP_STATUS_CAN_ID (IMU_CAN_ID + 0x100U)
#else
#define IMU_CAN_DLC_BYTES       16U
#endif

/**
  * @brief  Create and start the IMU acquisition/transmission task.
  * @note   Called once from main() before the scheduler starts.
  */
void IMU_StartTask(void);

/**
  * @brief  EXTI0 data-ready callback (called from HAL_GPIO_EXTI_Callback in
  *         interrupt context). Wakes the IMU task.
  */
void IMU_DataReadyFromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_TASK_H */
