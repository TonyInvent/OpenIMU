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

/**
  * @brief  CAN FD frame identifier used for IMU data.
  */
#define IMU_CAN_ID              0x01U

/**
  * @brief  Number of data bytes in one IMU CAN FD frame (16 bytes).
  */
#define IMU_CAN_DLC_BYTES       16U

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
