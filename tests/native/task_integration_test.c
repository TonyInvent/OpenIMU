/* Run the actual acquisition task and sync module with bounded mocked wakes. */
#include "sync_mock.h"
#include <setjmp.h>
#define __IMU_TASK_H
#define __ICM45686_H
#define IMU_CAN_ID 1U
#define IMU_CAN_DLC_BYTES 16U
#define GPIO_PIN_2 4U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define EXTI0_IRQn 6U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_DLC_BYTES_24 24U
#define FDCAN_PSR_BO 1U
#define FDCAN_CCCR_INIT 1U
#define ICM45686_STATUS_FIFO_FULL 1U
#define ICM45686_STATUS_RESET_DONE 2U
#define pdFALSE 0
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define HAL_GPIO_WritePin(a,b,c) ((void)(a),(void)(b),(void)(c))
#define HAL_GPIO_TogglePin(a,b) ((void)(a),(void)(b))
#define portYIELD_FROM_ISR(a) ((void)(a))
typedef int SPI_HandleTypeDef;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef int osPriority_t;
enum {osPriorityHigh=1};
typedef struct {const char *name;osPriority_t priority;uint32_t stack_size;} osThreadAttr_t;
typedef struct {int16_t accel[3],gyro[3],temp;} ICM45686_Data_t;
typedef struct {unsigned unused;} FDCAN_ProtocolStatusTypeDef;
typedef struct {unsigned unused;} FDCAN_ErrorCountersTypeDef;
SPI_HandleTypeDef hspi1;
static CanRegs cr;
FDCAN_HandleTypeDef hfdcan1={.Instance=&cr};
static jmp_buf task_done;
static unsigned wake_index,packet_index,pending_samples;
static unsigned fail_read,scenario;
static uint32_t ids[16],lengths[16];
static uint8_t payloads[16][48];
static unsigned sent;
static void osDelay(uint32_t ms){(void)ms;assert(!"unexpected fatal initialization");}
static TaskHandle_t osThreadNew(void(*fn)(void*),void *arg,const osThreadAttr_t *attr){(void)fn;(void)arg;(void)attr;return (void*)1;}
static void vTaskNotifyGiveFromISR(TaskHandle_t h,BaseType_t *wake){(void)h;(void)wake;}
static int ICM45686_Init(SPI_HandleTypeDef *h){(void)h;return HAL_OK;}
static int ICM45686_ConfigFifo(void){return HAL_OK;}
static uint16_t ICM45686_ReadFifoCount(void){return (uint16_t)pending_samples;}
static int ICM45686_ReadFifoPacket(ICM45686_Data_t *v,uint16_t *stamp){
    packet_index++;
    if(fail_read && packet_index==fail_read)return 1;
    *stamp=(uint16_t)(packet_index*1250U);*v=(ICM45686_Data_t){.accel={1,2,3},.gyro={4,5,6},.temp=7};
    if(pending_samples)pending_samples--;
    return HAL_OK;
}
static int ICM45686_ReadFifoStatus(uint8_t *status){*status=0;return HAL_OK;}
static int HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *h,unsigned a,unsigned b){(void)h;(void)a;(void)b;return HAL_OK;}
static int HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *h){(void)h;return HAL_OK;}
static int HAL_FDCAN_Start(FDCAN_HandleTypeDef *h){(void)h;return HAL_OK;}
static void HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *v){(void)h;(void)v;}
static void HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *v){(void)h;(void)v;}
#include "../../Core/Src/imu_sync.c"
static void queue_control(uint8_t op,unsigned seq){
    rx_header=(FDCAN_RxHeaderTypeDef){.Identifier=0x700,.FDFormat=FDCAN_FD_CAN,.DataLength=16,.RxTimestamp=TIM3->CNT};
    memset(rx_data,0,64);memcpy(rx_data,"OSYN",4);rx_data[4]=1;rx_data[5]=op;put32(rx_data+8,77);put32(rx_data+12,seq);rx_count=1;
}
static uint32_t ulTaskNotifyTake(int clear,uint32_t timeout){
    (void)clear;(void)timeout;
    if(wake_index==3)longjmp(task_done,1);
    wake_index++;tick=wake_index;TIM3->CNT=wake_index*1250U;TIM3->CCR3=TIM3->CNT;TIM3->SR=TIM_SR_CC3IF;DWT->CYCCNT=TIM3->CNT*100U;IMU_SyncTimerIRQ();
    pending_samples=(scenario==1 && wake_index==2)?3:1;
    if(wake_index==2)queue_control(1,1);
    if(wake_index==3)queue_control(3,2);
    return 1;
}
static int record_tx(FDCAN_TxHeaderTypeDef *h,uint8_t *data){
    assert(sent<16);ids[sent]=h->Identifier;lengths[sent]=h->DataLength;
    memcpy(payloads[sent],data,h->DataLength);sent++;return HAL_OK;
}
#include "../../Core/Src/imu_task.c"
int main(int argc,char **argv){
    scenario=(argc>1 && !strcmp(argv[1],"backlog"))?1:((argc>1 && !strcmp(argv[1],"spi-failure"))?2:0);
    if(scenario==2)fail_read=2;
    RCC->CRRCR=RCC_CRRCR_HSI48RDY;RNG->SR=RNG_SR_DRDY;RNG->DR=123;tx_hook=record_tx;
    (void)&HAL_FDCAN_GetRxFifoFillLevel;
    if(setjmp(task_done)==0)IMU_Task(NULL);
    assert(ids[0]==1 && lengths[0]==16);
    if(scenario==2){assert(sent==2 && ids[1]==1 && lengths[1]==16 && imu_spi_error_count==1);}
    else{
        assert(sent==4);assert(ids[1]==1 && lengths[1]==24);
        assert(ids[2]==0x711 && lengths[2]==48); /* DATA precedes ANCHOR in TX FIFO */
        assert(ids[3]==1 && lengths[3]==16);
        assert(payloads[2][1]==(scenario==1?1:3));
        assert(payloads[1][2]==0 && payloads[1][3]==1 && payloads[1][15]==7);
        if(scenario==1)assert(imu_fifo_backlog_count==2);
    }
    printf("actual IMU_Task integration scenario %u: PASS\n",scenario);return 0;
}
