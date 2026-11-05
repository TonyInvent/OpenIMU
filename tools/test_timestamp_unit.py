#!/usr/bin/env python3
"""Real optional task/clock code under a host HAL/RTOS model, ASan + UBSan."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent
MAIN = r'''
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { uint32_t PSR, CCCR; } FDCAN_Regs;
typedef struct { FDCAN_Regs *Instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,
    BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker; } FDCAN_TxHeaderTypeDef;
typedef struct { int unused; } FDCAN_ProtocolStatusTypeDef;
typedef struct { int unused; } FDCAN_ErrorCountersTypeDef;
typedef struct { uint32_t VAL; } SysTick_Type;
extern SysTick_Type *SysTick;
typedef enum { GPIO_PIN_RESET=0, GPIO_PIN_SET=1 } GPIO_PinState;
#define GPIOB ((void *)2)
#define GPIO_PIN_0 1U
#define GPIO_PIN_2 4U
#define EXTI0_IRQn 6
#define FDCAN_DLC_BYTES_16 16U
#define FDCAN_DLC_BYTES_32 32U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_ON 1U
#define FDCAN_FD_CAN 1U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_PSR_BO 128U
#define FDCAN_CCCR_INIT 1U
void HAL_GPIO_WritePin(void *,uint16_t,GPIO_PinState);
void HAL_GPIO_TogglePin(void *,uint16_t);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
void HAL_NVIC_DisableIRQ(int);
void HAL_NVIC_ClearPendingIRQ(int);
void HAL_NVIC_EnableIRQ(int);
void HAL_NVIC_SetPriority(int,unsigned,unsigned);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *,const FDCAN_TxHeaderTypeDef *,uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *,FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *,FDCAN_ErrorCountersTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
#endif
'''
OS = r'''
#ifndef TEST_OS_H
#define TEST_OS_H
#include <stdint.h>
typedef int osPriority_t;
enum {osPriorityHigh=40};
typedef struct {const char *name;osPriority_t priority;uint32_t stack_size;} osThreadAttr_t;
void *osThreadNew(void (*)(void *),void *,const osThreadAttr_t *);
int osDelay(uint32_t);
#endif
'''
TASK = r'''
#ifndef TEST_TASK_H
#define TEST_TASK_H
#include <stdint.h>
typedef void *TaskHandle_t;
typedef int BaseType_t;
#define pdFALSE 0
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define portYIELD_FROM_ISR(x) ((void)(x))
uint32_t ulTaskNotifyTake(BaseType_t,uint32_t);
void vTaskNotifyGiveFromISR(TaskHandle_t,BaseType_t *);
BaseType_t xTaskNotifyGive(TaskHandle_t);
#endif
'''
HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "imu_task.c"
SPI_HandleTypeDef hspi1;
static FDCAN_Regs registers;
FDCAN_HandleTypeDef hfdcan1={&registers};
static SysTick_Type systick={12345};
SysTick_Type *SysTick=&systick;
static uint32_t ticks=2000;
static unsigned notifications,flushes,inits,enables,irq_disabled,fail_status,fail_data;
static ICM45686_FifoSample_t samples[100];
static unsigned sample_count,sample_index;
static ICM45686_FifoResult_t next_result=ICM45686_FIFO_EMPTY;
static struct { FDCAN_TxHeaderTypeDef header;uint8_t bytes[32]; } sent[256];
static unsigned sent_count;
void HAL_GPIO_WritePin(void *p,uint16_t n,GPIO_PinState v){(void)p;(void)n;(void)v;}
void HAL_GPIO_TogglePin(void *p,uint16_t n){(void)p;(void)n;}
uint32_t HAL_GetTick(void){return ticks;}
uint32_t HAL_GetUIDw0(void){return 1;}
uint32_t HAL_GetUIDw1(void){return 2;}
uint32_t HAL_GetUIDw2(void){return 3;}
void HAL_NVIC_DisableIRQ(int n){assert(n==EXTI0_IRQn);irq_disabled++;}
void HAL_NVIC_ClearPendingIRQ(int n){assert(n==EXTI0_IRQn);}
void HAL_NVIC_EnableIRQ(int n){assert(n==EXTI0_IRQn);assert(irq_disabled);irq_disabled--;}
void HAL_NVIC_SetPriority(int n,unsigned a,unsigned b){(void)n;(void)a;(void)b;}
void *osThreadNew(void (*f)(void *),void *a,const osThreadAttr_t *v){(void)f;(void)a;assert(v->stack_size==2048);return (void*)1;}
int osDelay(uint32_t ms){ticks+=ms;return 0;}
uint32_t ulTaskNotifyTake(BaseType_t a,uint32_t b){(void)a;(void)b;return 1;}
void vTaskNotifyGiveFromISR(TaskHandle_t h,BaseType_t *w){assert(h);*w=0;notifications++;}
BaseType_t xTaskNotifyGive(TaskHandle_t h){assert(h);notifications++;return 1;}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,const FDCAN_TxHeaderTypeDef *v,uint8_t *p){
    assert(h==&hfdcan1);
    if(v->Identifier==IMU_TIMESTAMP_STATUS_CAN_ID && fail_status){fail_status--;return HAL_ERROR;}
    if(v->Identifier==IMU_CAN_ID && fail_data){fail_data--;return HAL_ERROR;}
    assert(sent_count<256 && v->DataLength<=32);sent[sent_count].header=*v;
    memcpy(sent[sent_count++].bytes,p,v->DataLength);return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *s){(void)h;(void)s;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *s){(void)h;(void)s;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b){(void)h;(void)a;(void)b;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *h){(void)h;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h){(void)h;return HAL_OK;}
HAL_StatusTypeDef ICM45686_Init(SPI_HandleTypeDef *p){assert(p==&hspi1);inits++;return HAL_OK;}
ICM45686_FifoResult_t ICM45686_FifoEnable(void){enables++;return ICM45686_FIFO_OK;}
ICM45686_FifoResult_t ICM45686_FifoFlush(void){flushes++;sample_index=sample_count;return ICM45686_FIFO_OK;}
ICM45686_FifoResult_t ICM45686_FifoRead(ICM45686_FifoSample_t *s){
    if(sample_index<sample_count){*s=samples[sample_index++];return ICM45686_FIFO_OK;}
    ICM45686_FifoResult_t r=next_result;next_result=ICM45686_FIFO_EMPTY;return r;
}
static void load(unsigned count,uint16_t start){sample_count=count;sample_index=0;
    for(unsigned i=0;i<count;i++)samples[i]=(ICM45686_FifoSample_t){
        .data={.accel={-1,32767,-32768},.gyro={123,-456,789},.temp=-1234},
        .timestamp_raw=(uint16_t)(start+i*1250U),.header=0x78};
}
static uint32_t u32(const uint8_t*p){return(uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static uint64_t u64(const uint8_t*p){uint64_t v=0;for(unsigned i=0;i<8;i++)v=v*256+p[i];return v;}
static FDCAN_TxHeaderTypeDef header={.Identifier=IMU_CAN_ID,.DataLength=FDCAN_DLC_BYTES_32};
int main(void){
    IMU_Timestamp_t clock;uint64_t t;IMU_TimestampReset(&clock);
    assert(IMU_TimestampAdvance(&clock,65000,&t)&&t==0);
    assert(IMU_TimestampAdvance(&clock,714,&t)&&t==1250);
    uint16_t raw=714;
    for(unsigned i=0;i<65540;i++){raw=(uint16_t)(raw+1250U);assert(IMU_TimestampAdvance(&clock,raw,&t));}
    assert(t==UINT64_C(65541)*1250);
    assert(!IMU_TimestampAdvance(&clock,raw,&t));
    assert(!IMU_TimestampAdvance(&clock,(uint16_t)(raw+20000U),&t));
    clock.time_us=UINT64_MAX-10;assert(!IMU_TimestampAdvance(&clock,(uint16_t)(raw+1250),&t));
    assert(!IMU_TimestampAdvance(NULL,0,&t));
    puts("PASS 1us raw16 multi-wrap unfolding, epoch-relative origin, invalid steps and u64 overflow rejection");
    uint8_t bytes[IMU_CAN_DLC_BYTES];uint16_t sequence=0;imu_task_handle=(void*)1;
    IMU_BeginTimestampEpoch(TIMESTAMP_EPOCH_BOOT);load(2,65000);
    IMU_DrainTimestampedFifo(&header,bytes,&sequence);assert(sent_count==3);
    assert(sent[0].header.Identifier==IMU_TIMESTAMP_STATUS_CAN_ID && sent[0].bytes[7]==32);
    unsigned first=1;uint8_t legacy[16];IMU_PackFrame(legacy,0,&samples[0].data);
    assert(!memcmp(legacy,sent[first].bytes,16));assert(u64(sent[first].bytes+16)==0);
    assert(u64(sent[first+1].bytes+16)==1250);assert(u32(sent[first].bytes+24)==1);
    uint32_t boot=u32(sent[first].bytes+28);assert(boot && boot==u32(sent[first+1].bytes+28));
    assert(sent[first].header.DataLength==32 && sequence==2 && imu_sample_count==2);
    puts("PASS original 16-byte prefix, appended BE64 hardware time and per-frame epoch/boot identity");
    unsigned before=sent_count;uint32_t old_epoch=imu_timestamp_epoch;
    next_result=ICM45686_FIFO_OVERFLOW;IMU_DrainTimestampedFifo(&header,bytes,&sequence);
    assert(imu_timestamp_epoch==old_epoch+1 && imu_fifo_overflow_count==1 && flushes==1 && notifications==1);
    load(1,50000);IMU_DrainTimestampedFifo(&header,bytes,&sequence);
    assert(sent_count==before+2 && u64(sent[before+1].bytes+16)==0);
    assert(u32(sent[before+1].bytes+24)!=u32(sent[first].bytes+24));
    assert(u32(sent[before+1].bytes+28)==boot); /* Discard ALL status: data still distinguishes epochs. */
    puts("PASS FIFO full creates new clock epoch; data identity remains explicit when status frames are discarded");
    before=sent_count;IMU_BeginTimestampEpoch(TIMESTAMP_EPOCH_SPI);load(2,30000);fail_status=1;
    IMU_DrainTimestampedFifo(&header,bytes,&sequence);assert(sent_count==before+2);
    assert(sent[before].header.Identifier==IMU_TIMESTAMP_STATUS_CAN_ID);
    assert(u64(sent[before+1].bytes+16)==1250); /* first sample intentionally not sent */
    next_result=ICM45686_FIFO_CONFIG_ERROR;IMU_DrainTimestampedFifo(&header,bytes,&sequence);
    assert(inits==1 && enables==1 && !irq_disabled);
    puts("PASS metadata enqueue rejection, source time remains honest, hardware-reset recovery reinitializes sensor");
    load(40,1000);unsigned wake_before=notifications;
    IMU_DrainTimestampedFifo(&header,bytes,&sequence);assert(sample_index==32 && notifications==wake_before+1);
    IMU_DrainTimestampedFifo(&header,bytes,&sequence);assert(sample_index==40);
    load(1,samples[39].timestamp_raw);wake_before=notifications;
    IMU_DrainTimestampedFifo(&header,bytes,&sequence);assert(imu_timestamp_error_count==1 && notifications==wake_before+1);
    puts("PASS bounded FIFO draining self-wakes; duplicate hardware timestamps flush into a new epoch");
    puts("5/5 PASS (ASan + UBSan; real task and clock sources)");return 0;
}
'''


def main():
    compiler = shutil.which('gcc')
    if not compiler:
        raise SystemExit('Host GCC required')
    with tempfile.TemporaryDirectory(prefix='openimu-timestamp-test-') as tmp:
        work = Path(tmp)
        for name in ('imu_task.c', 'imu_timestamp.c'):
            shutil.copyfile(ROOT / 'Core/Src' / name, work / name)
        for name in ('imu_task.h', 'imu_timestamp.h', 'imu_config.h', 'icm45686_fifo.h', 'icm45686.h'):
            shutil.copyfile(ROOT / 'Core/Inc' / name, work / name)
        for name, content in [('main.h', MAIN), ('cmsis_os.h', OS), ('task.h', TASK), ('test.c', HARNESS)]:
            (work / name).write_text(content)
        exe = work / 'test'
        subprocess.run([compiler, '-std=c11', '-DIMU_ENABLE_SOURCE_TIMESTAMP=1', '-Wall', '-Wextra',
                        '-Werror', '-g', '-O1', '-fno-pie', '-no-pie', '-fsanitize=address,undefined',
                        '-fno-omit-frame-pointer', '-I', str(work), str(work / 'imu_timestamp.c'),
                        str(work / 'test.c'), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], env={**os.environ, 'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1',
                                       'UBSAN_OPTIONS': 'halt_on_error=1'}, check=True)


if __name__ == '__main__':
    main()
