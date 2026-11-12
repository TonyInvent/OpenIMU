#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <setjmp.h>
#define __MAIN_H
#define __IMU_TASK_H
#define __ICM45686_H
#define IMU_CAN_ID 1U
#define IMU_CAN_DLC_BYTES 16U
#define GPIOB 0
#define GPIO_PIN_0 1U
#define GPIO_PIN_2 4U
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define EXTI0_IRQn 6
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_NO_TX_EVENTS 0
#define FDCAN_PSR_BO 128U
#define FDCAN_CCCR_INIT 1U
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_DLC_BYTES_16 16
#define FDCAN_BRS_ON 1
#define FDCAN_FD_CAN 1
#define ICM45686_STATUS_FIFO_FULL 1U
#define ICM45686_STATUS_RESET_DONE 128U
#define pdFALSE 0
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define portYIELD_FROM_ISR(a) ((void)(a))
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef int SPI_HandleTypeDef;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef int osPriority_t;
enum {osPriorityHigh=1};
typedef struct {const char *name;osPriority_t priority;uint32_t stack_size;} osThreadAttr_t;
typedef struct {int16_t accel[3],gyro[3],temp;} ICM45686_Data_t;
typedef struct {uint32_t PSR,CCCR;} CanRegs;
typedef struct {CanRegs *Instance;} FDCAN_HandleTypeDef;
typedef struct {uint32_t Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker;} FDCAN_TxHeaderTypeDef;
typedef struct {unsigned unused;} FDCAN_ProtocolStatusTypeDef;
typedef struct {unsigned unused;} FDCAN_ErrorCountersTypeDef;
SPI_HandleTypeDef hspi1;
static CanRegs cr;
FDCAN_HandleTypeDef hfdcan1={&cr};
static jmp_buf done;
static unsigned scenario,wake,reads,pending,sent,format_errors;
static uint16_t sent_stamps[3];
static void HAL_GPIO_WritePin(int p,unsigned pin,int v){(void)p;(void)pin;(void)v;}
static void HAL_GPIO_TogglePin(int p,unsigned pin){(void)p;(void)pin;}
static uint32_t HAL_GetTick(void){return wake*100;}
static void osDelay(uint32_t ms){(void)ms;assert(!"unexpected fatal init");}
static TaskHandle_t osThreadNew(void(*f)(void*),void *a,const osThreadAttr_t *t){(void)f;(void)a;(void)t;return (void*)1;}
static void vTaskNotifyGiveFromISR(TaskHandle_t t,BaseType_t *w){(void)t;(void)w;}
static void HAL_NVIC_SetPriority(int n,unsigned a,unsigned b){(void)n;(void)a;(void)b;}
static void HAL_NVIC_ClearPendingIRQ(int n){(void)n;}
static void HAL_NVIC_EnableIRQ(int n){(void)n;}
static int ICM45686_Init(SPI_HandleTypeDef *h){(void)h;return HAL_OK;}
static int ICM45686_ConfigFifo(void){return HAL_OK;}
static int ICM45686_ReadFifoCountChecked(uint16_t *n){*n=(uint16_t)pending;return scenario==2&&wake==1?HAL_ERROR:HAL_OK;}
static int ICM45686_ReadFifoPacket(ICM45686_Data_t *v,uint16_t *stamp){
    ++reads;assert(pending>0);--pending;
    if((scenario==3||scenario==4)&&wake==2){if(scenario==3)++format_errors;return HAL_ERROR;}
    if(scenario==6&&wake==1){++format_errors;return HAL_ERROR;}
    *v=(ICM45686_Data_t){.accel={1,2,3},.gyro={4,5,6},.temp=7};*stamp=(uint16_t)(reads*1250);return HAL_OK;
}
static int ICM45686_ReadFifoStatus(uint8_t *n){*n=0;return scenario==7&&wake==2?HAL_ERROR:HAL_OK;}
static int HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *h,unsigned a,unsigned b){(void)h;(void)a;(void)b;return HAL_OK;}
static int HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *h){(void)h;return HAL_OK;}
static int HAL_FDCAN_Start(FDCAN_HandleTypeDef *h){(void)h;return HAL_OK;}
static void HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *v){(void)h;(void)v;}
static void HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *v){(void)h;(void)v;}
static uint32_t ulTaskNotifyTake(int clear,uint32_t timeout){
    (void)clear;(void)timeout;if(wake==3)longjmp(done,1);++wake;
    pending=(scenario==1&&wake==1)?0:((scenario==3||scenario==4||scenario==5)&&wake==2?3:1);return 1;
}
static int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,FDCAN_TxHeaderTypeDef *hdr,uint8_t *bytes){
    (void)h;assert(sent<3&&hdr->Identifier==1&&hdr->DataLength==16);
    assert(bytes[2]==0&&bytes[3]==1&&bytes[15]==7);
    sent_stamps[sent++]=(uint16_t)(((uint16_t)bytes[0]<<8)|bytes[1]);return HAL_OK;
}
#include "../../Core/Src/imu_task.c"
int main(void){
    for(scenario=0;scenario<8;++scenario){
        wake=reads=pending=sent=format_errors=0;
        imu_sample_count=imu_spi_error_count=imu_fifo_backlog_count=0;
        if(setjmp(done)==0)IMU_Task(NULL);
        assert(sent==((scenario==0||scenario==5)?3:2));
        assert(imu_spi_error_count==((scenario==2||scenario==3||scenario==4||scenario==6||scenario==7)?1:0));
        assert(format_errors==((scenario==3||scenario==6)?1:0));
        if(scenario==1)assert(reads==2);
        if(scenario==2)assert(reads==2);
        if(scenario==3||scenario==4)assert(reads==3);
        if(scenario==5)assert(reads==5&&imu_fifo_backlog_count==2&&sent_stamps[1]==5000);
        printf("PASS actual IMU task scenario %u\n",scenario);
    }
    return 0;
}
