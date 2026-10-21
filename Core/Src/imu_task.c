/**
  ******************************************************************************
  * @file    imu_task.c
  * @brief   FreeRTOS task reading ICM-45686 samples on the INT1 data-ready
  *          interrupt and streaming them over CAN FD (ID 0x01) at 800 Hz.
  *
  *          Flow:
  *            ICM-45686 INT1 (800 Hz data-ready, active-high pulse)
  *                -> PB0 / EXTI0 (rising edge)
  *                -> HAL_GPIO_EXTI_Callback()
  *                -> vTaskNotifyGiveFromISR()
  *                -> IMU task reads 14 bytes over SPI and queues one 16-byte
  *                   CAN FD frame.
  *
  *          CAN FD frame (16 bytes, big-endian multi-byte fields):
  *            [0]    sequence high
  *            [1]    sequence low
  *            [2:3]  accel X  (raw int16)
  *            [4:5]  accel Y
  *            [6:7]  accel Z
  *            [8:9]  gyro X   (raw int16)
  *            [10:11]gyro Y
  *            [12:13]gyro Z
  *            [14:15]temperature (raw int16)
  ******************************************************************************
  */
#include "imu_task.h"
#include "icm45686.h"
#include "main.h"
#include "task.h"

/* External peripheral handles from main.c */
extern SPI_HandleTypeDef   hspi1;
extern FDCAN_HandleTypeDef hfdcan1;

/* ------------------------------------------------------------------------- */
/* Task configuration                                                        */
/* ------------------------------------------------------------------------- */
#define IMU_TASK_STACK_BYTES      (256U * 4U)   /* 256 words = 1024 bytes */

static const osThreadAttr_t imu_task_attributes = {
    .name       = "imuTask",
    .priority   = (osPriority_t) osPriorityHigh,
    .stack_size = IMU_TASK_STACK_BYTES
};

/* ------------------------------------------------------------------------- */
/* Private variables                                                         */
/* ------------------------------------------------------------------------- */
static TaskHandle_t imu_task_handle = NULL;   /* used for ISR -> task notify */

/* Diagnostics (inspect in the debugger) */
volatile uint32_t imu_sample_count = 0U;      /* total samples transmitted   */
volatile uint32_t imu_missed_count  = 0U;     /* data-ready overruns         */
volatile uint32_t imu_can_error_count = 0U;   /* CAN queue failures          */
volatile uint32_t imu_spi_error_count = 0U;   /* SPI read failures           */

/* ------------------------------------------------------------------------- */
/* Private function prototypes                                               */
/* ------------------------------------------------------------------------- */
static void IMU_Task(void *argument);

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

/**
  * @brief  Pack one sample into the 16-byte CAN FD payload (big-endian).
  */
static void IMU_PackFrame(uint8_t *dst, uint16_t seq, const ICM45686_Data_t *d)
{
    dst[0]  = (uint8_t)(seq >> 8);
    dst[1]  = (uint8_t)(seq & 0xFFU);

    dst[2]  = (uint8_t)((uint16_t)d->accel[0] >> 8);
    dst[3]  = (uint8_t)((uint16_t)d->accel[0] & 0xFFU);
    dst[4]  = (uint8_t)((uint16_t)d->accel[1] >> 8);
    dst[5]  = (uint8_t)((uint16_t)d->accel[1] & 0xFFU);
    dst[6]  = (uint8_t)((uint16_t)d->accel[2] >> 8);
    dst[7]  = (uint8_t)((uint16_t)d->accel[2] & 0xFFU);

    dst[8]  = (uint8_t)((uint16_t)d->gyro[0] >> 8);
    dst[9]  = (uint8_t)((uint16_t)d->gyro[0] & 0xFFU);
    dst[10] = (uint8_t)((uint16_t)d->gyro[1] >> 8);
    dst[11] = (uint8_t)((uint16_t)d->gyro[1] & 0xFFU);
    dst[12] = (uint8_t)((uint16_t)d->gyro[2] >> 8);
    dst[13] = (uint8_t)((uint16_t)d->gyro[2] & 0xFFU);

    dst[14] = (uint8_t)((uint16_t)d->temp >> 8);
    dst[15] = (uint8_t)((uint16_t)d->temp & 0xFFU);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void IMU_StartTask(void)
{
    imu_task_handle = osThreadNew(IMU_Task, NULL, &imu_task_attributes);
}

/**
  * @brief  Called from EXTI0 ISR context; wakes the IMU task.
  */
void IMU_DataReadyFromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (imu_task_handle != NULL) {
        vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  HAL GPIO EXTI callback (called from EXTI0_IRQHandler via
  *         HAL_GPIO_EXTI_IRQHandler()).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        IMU_DataReadyFromISR();
    }
}

/* ------------------------------------------------------------------------- */
/* Task body                                                                 */
/* ------------------------------------------------------------------------- */

static void IMU_Task(void *argument)
{
    (void)argument;

    FDCAN_TxHeaderTypeDef TxHeader = {0};
    ICM45686_Data_t       imu      = {0};
    uint8_t               tx_data[IMU_CAN_DLC_BYTES];
    uint16_t              seq = 0U;

    /* 1. Initialise the IMU and start the FDCAN peripheral */
    if (ICM45686_Init(&hspi1) != HAL_OK) {
        /* IMU not present / miswired. Spin here; inspect in the debugger. */
        for (;;) {
        }
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        for (;;) {
        }
    }

    /* 2. Prepare the (constant) CAN FD TX header */
    TxHeader.Identifier          = IMU_CAN_ID;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = FDCAN_DLC_BYTES_16;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_ON;
    TxHeader.FDFormat            = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0U;

    /* 3. Enable the INT1 / EXTI0 data-ready interrupt (FreeRTOS-safe priority) */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    /* 4. Acquisition loop: one CAN FD frame per data-ready pulse (800 Hz) */
    for (;;) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (notified > 1U) {
            imu_missed_count += (notified - 1U);
        }

        if (ICM45686_ReadData(&imu) != HAL_OK) {
            imu_spi_error_count++;
            continue;
        }

        IMU_PackFrame(tx_data, seq++, &imu);

        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, tx_data) != HAL_OK) {
            imu_can_error_count++;
            /* Frame dropped due to a full TX FIFO / bus overload. */
        } else {
            imu_sample_count++;
        }
    }
}
