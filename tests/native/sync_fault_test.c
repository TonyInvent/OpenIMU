/* Failure injection against the actual C module, including register side effects. */
#include "sync_mock.h"
#include "../../Core/Src/imu_sync.c"
#include <stdlib.h>
static CanRegs cr;
static FDCAN_HandleTypeDef can={.Instance=&cr};
static void initialize(void) {
    RCC->CRRCR=RCC_CRRCR_HSI48RDY;RNG->SR=RNG_SR_DRDY;RNG->DR=123;
    assert(IMU_SyncInit(&can));
}
static void advance(uint32_t us) {
    uint64_t total=timer_high+TIM3->CNT+us;
    timer_high=total&~UINT64_C(65535);TIM3->CNT=total&65535;
    tick=(uint32_t)(total/1000);DWT->CYCCNT+=us*100;
}
static void queue(uint8_t op,uint32_t s,uint32_t q) {
    rx_header=(FDCAN_RxHeaderTypeDef){.Identifier=0x700,.FDFormat=FDCAN_FD_CAN,.DataLength=16,.RxTimestamp=TIM3->CNT};
    memset(rx_data,0,64);memcpy(rx_data,"OSYN",4);rx_data[4]=1;rx_data[5]=op;put32(rx_data+8,s);put32(rx_data+12,q);rx_count=1;
}
static void start(void) {queue(1,77,1);IMU_SyncPoll();assert(active);}
static void preempt_rx(void) {advance(66000);rx_hook=NULL;}
static void overflow_rx(void) {cr.IR|=FDCAN_IR_RF0L;rx_hook=NULL;}
static void wrap_before_status(char operation) {
    if(operation=='S') {TIM3->CNT=0;TIM3->SR|=TIM_SR_UIF;register_hook=NULL;}
}
static unsigned reads;
static void capture_after_status(char operation) {
    if(operation=='N' && ++reads==1) {TIM3->CCR3=999;TIM3->SR|=TIM_SR_CC3IF;register_hook=NULL;}
}
int main(int argc,char **argv) {
    assert(argc==2);const char *test=argv[1];
    if(!strncmp(test,"config-fail-",12)) {
        config_failure=(unsigned)atoi(test+12);assert(config_failure>=1 && config_failure<=4);
        RCC->CRRCR=RCC_CRRCR_HSI48RDY;RNG->SR=RNG_SR_DRDY;RNG->DR=123;
        assert(!IMU_SyncInit(&can));assert(!ready && !active);
        assert(TIM3->CR1==0 && TIM3->DIER==0);
    } else if(!strcmp(test,"rng-timeout")) {
        tick_step=1;assert(!IMU_SyncInit(&can));assert(!ready && !active);assert(tick<100);
    } else if(!strcmp(test,"reinit-failure")) {
        initialize();start();config_stage=0;config_failure=1;RNG->DR=456;
        assert(!IMU_SyncInit(&can));assert(!ready && !active && !pending);
        uint8_t data[24]={0};assert(!IMU_SyncExtend(data));assert(TIM3->CR1==0);
    } else if(!strcmp(test,"rx-preemption")) {
        initialize();queue(1,77,1);rx_hook=preempt_rx;IMU_SyncPoll();assert(!active);
        IMU_SyncPoll(); /* normal scheduler resumes and proves FIFO empty */
        queue(1,77,2);IMU_SyncPoll();assert(active);
    } else if(!strcmp(test,"rx-failure-age")) {
        initialize();queue(1,77,1);rx_result=1;
        for(unsigned i=0;i<66;i++){advance(1000);IMU_SyncPoll();}
        rx_result=0;IMU_SyncPoll();assert(!active);
        queue(1,77,2);IMU_SyncPoll();assert(active);
    } else if(!strcmp(test,"rx-overflow-during-read")) {
        initialize();queue(1,77,1);rx_hook=overflow_rx;IMU_SyncPoll();assert(!active);
        assert(!(cr.IR&FDCAN_IR_RF0L));start();
    } else if(!strcmp(test,"rx-overflow-before-report")) {
        initialize();start();uint32_t old=epoch;cr.IR|=FDCAN_IR_RF0L;
        IMU_SyncReport(0,false);assert(epoch!=old && tx_count==1 && tx_data[1]==0);
        assert(!(cr.IR&FDCAN_IR_RF0L));
    } else if(!strcmp(test,"revocation-retry")) {
        initialize();start();advance(1000);queue(2,77,1);IMU_SyncPoll();assert(sync_ambiguous);
        tx_free=1;IMU_SyncReport(0,false);assert(pending && tx_count==0);
        queue(2,77,2);IMU_SyncPoll();assert(sequence==1);
        tx_free=3;tx_result=1;IMU_SyncReport(0,false);assert(pending && tx_count==0);
        tx_result=0;IMU_SyncReport(0,false);assert(!pending && tx_count==1 && tx_data[1]==0);
        queue(2,77,2);IMU_SyncPoll();assert(sequence==2);
    } else if(!strcmp(test,"overflow-read-race")) {
        initialize();TIM3->CNT=65535;register_hook=wrap_before_status;
        assert(timer_now()==65536);assert(TIM3->SR&TIM_SR_UIF);
        IMU_SyncTimerIRQ();assert(timer_high==65536);assert(!(TIM3->SR&TIM_SR_UIF));
    } else if(!strcmp(test,"capture-arrival-race")) {
        initialize();TIM3->CNT=1000;DWT->CYCCNT=100000;register_hook=capture_after_status;
        IMU_SyncTimerIRQ();assert(capture_seq==0);assert(TIM3->SR&TIM_SR_CC3IF);
        IMU_SyncTimerIRQ();assert(capture_seq==1 && capture_time==999);assert(!(TIM3->SR&TIM_SR_CC3IF));
    } else if(!strcmp(test,"capture-across-wrap")) {
        initialize();irq_last_time=65000;irq_last_cycles=6500000;
        TIM3->CNT=2;TIM3->CCR3=65534;TIM3->SR=TIM_SR_UIF|TIM_SR_CC3IF;
        DWT->CYCCNT=6553800;IMU_SyncTimerIRQ();assert(capture_time==65534 && timer_high==65536);
        assert(!(TIM3->SR&(TIM_SR_UIF|TIM_SR_CC3IF)));
    } else if(!strcmp(test,"stale-capture-one-wrap")) {
        initialize();start();IMU_SyncSourceSample(1);
        TIM3->CNT=100000U&65535U;TIM3->CCR3=1;TIM3->SR=TIM_SR_UIF|TIM_SR_CC3IF;
        DWT->CYCCNT=10000000;IMU_SyncTimerIRQ();
        assert(!clock_fault && overrun_count>0 && capture_seq==0);
        IMU_SyncReport(IMU_SyncCaptureToken(),true);assert(tx_count==1 && tx_data[1]==1);
    } else if(!strcmp(test,"first-irq-lost-wrap")) {
        initialize();TIM3->CNT=140000U&65535U;TIM3->SR=TIM_SR_UIF;
        DWT->CYCCNT=14000000;IMU_SyncTimerIRQ();assert(clock_fault);
        queue(1,77,1);IMU_SyncPoll();assert(!active);
    } else if(!strcmp(test,"capture-token-reuse")) {
        initialize();start();IMU_SyncSourceSample(1000);
        TIM3->CNT=1000;TIM3->CCR3=1000;TIM3->SR=TIM_SR_CC3IF;
        DWT->CYCCNT=100000;IMU_SyncTimerIRQ();
        assert(IMU_SyncCaptureToken()==1);assert(IMU_SyncCaptureToken()==0);
        IMU_SyncReport(0,true);assert(tx_data[1]==1); /* CAN valid; sample pair invalid */
    } else if(!strcmp(test,"capture-overrun")) {
        initialize();start();IMU_SyncSourceSample(1000);
        TIM3->CNT=1000;TIM3->CCR3=1000;TIM3->SR=TIM_SR_CC3IF|TIM_SR_CC3OF;
        DWT->CYCCNT=100000;IMU_SyncTimerIRQ();assert(overrun_count==1);
        assert(!(TIM3->SR&(TIM_SR_CC3IF|TIM_SR_CC3OF)));
        IMU_SyncReport(IMU_SyncCaptureToken(),true);assert(tx_data[1]==1);
    } else if(!strcmp(test,"lease-tick-wrap")) {
        initialize();tick=UINT32_MAX-500;last_poll_ms=tick;start();
        tick=498;IMU_SyncPoll();assert(active); /* 999 ms elapsed */
        tick=499;IMU_SyncPoll();assert(!active); /* exactly 1000 ms */
    } else if(!strcmp(test,"source-two-hours")) {
        initialize();start();IMU_SyncSourceSample(0);
        uint8_t data[24];memset(data,0xa5,sizeof(data));uint32_t q=1;
        for(uint32_t i=1;i<=5760000U;i++) {
            advance(1250);IMU_SyncPoll();
            if(i%80U==0) {queue(2,77,++q);IMU_SyncPoll();}
            uint64_t expected=(uint64_t)i*1250U;
            IMU_SyncSourceSample((uint16_t)expected);
            assert(source_time==expected && epoch==123);
            assert(IMU_SyncExtend(data));assert(be32(data+16)==(uint32_t)expected);
            for(unsigned j=0;j<16;j++)assert(data[j]==0xa5);
        }
        assert(source_time==UINT64_C(7200000000));
    } else if(!strcmp(test,"malformed-controls")) {
        initialize();
        for(unsigned i=0;i<10000;i++) {
            advance(1000);IMU_SyncPoll();queue(1,77,i);
            switch(i%8) {
                case 0:rx_data[0]='X';break;case 1:rx_data[4]=2;break;
                case 2:rx_data[6]=1;break;case 3:rx_data[5]=4;break;
                case 4:put32(rx_data+8,0);break;case 5:rx_header.DataLength=48;break;
                case 6:rx_header.Identifier=1;break;case 7:rx_header.FDFormat=0;break;
            }
            IMU_SyncPoll();assert(!active && !pending);
        }
    } else {fprintf(stderr,"unknown case %s\n",test);return 2;}
    printf("fault injection %s: PASS\n",test);return 0;
}
