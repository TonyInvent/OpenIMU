#include "sync_mock.h"
#include "../../Core/Src/imu_sync.c"
static void command(uint8_t op,uint32_t s,uint32_t seq) {
    tick++;TIM3->CNT+=1000;last_poll_ms=tick;
    rx_header=(FDCAN_RxHeaderTypeDef){.Identifier=0x700,.FDFormat=FDCAN_FD_CAN,.DataLength=16,.RxTimestamp=TIM3->CNT};
    memset(rx_data,0,64);memcpy(rx_data,"OSYN",4);rx_data[4]=1;rx_data[5]=op;put32(rx_data+8,s);put32(rx_data+12,seq);rx_count=1;IMU_SyncPoll();
}
int main(int argc, char **argv) {
    (void)argv;
    CanRegs cr={0};FDCAN_HandleTypeDef can={.Instance=&cr};
    rcc_regs.CRRCR=RCC_CRRCR_HSI48RDY;rng_regs.SR=RNG_SR_DRDY;rng_regs.DR=123;
    if(argc>1) {
        rng_regs.SR|=RNG_SR_CECS;
        assert(!IMU_SyncInit(&can));
        IMU_SyncSourceSample(12);command(1,77,1);
        uint8_t fallback[24]={0};assert(!active && !IMU_SyncExtend(fallback));
        puts("RNG failure: synchronization unavailable, legacy format retained PASS");return 0;
    }
    assert(IMU_SyncInit(&can));assert(TIM3->PSC==99);assert(!active);
    uint8_t data[24];memset(data,0xa5,sizeof(data));IMU_SyncSourceSample(65530);
    assert(!IMU_SyncExtend(data));command(2,77,0);assert(!active);
    command(1,77,1);assert(active);assert(IMU_SyncExtend(data));
    for(unsigned i=0;i<16;i++)assert(data[i]==0xa5);
    assert(be32(data+16)==65530 && be32(data+20)==123);
    uint32_t lease=lease_ms;tick++;TIM3->CNT+=1000;last_poll_ms=tick;
    rx_data[5]=2;rx_count=1;IMU_SyncPoll();assert(lease_ms==lease);
    command(1,78,2);assert(session==77 && lease_ms==lease);
    command(2,77,2);assert(sequence==2 && lease_ms>lease);
    IMU_SyncSourceSample(5);assert(source_time==65541);
    capture_seq=1;capture_time=sync_time+10;TIM3->SR=0;IMU_SyncReport(1,true);
    assert(tx_count==1 && tx_data[0]==1 && tx_data[1]==3 && tx_data[2]==1);
    assert(be32(tx_data+4)==77 && be32(tx_data+8)==2 && be32(tx_data+12)==123);
    assert(be32(tx_data+36)==65541);
    printf("ANCHOR_HEX=");for(unsigned i=0;i<48;i++)printf("%02x",tx_data[i]);puts("");
    lease=lease_ms;command(2,77,2);assert(lease_ms==lease && sync_ambiguous);
    IMU_SyncReport(1,true);assert(tx_count==2 && tx_data[1]==0);
    printf("INVALID_HEX=");for(unsigned i=0;i<48;i++)printf("%02x",tx_data[i]);puts("");
    /* Equal low16 does not mean the same physical event after a full wrap. */
    sync_ambiguous=false;pending=false;
    timer_high=(sync_time+65536U)&~UINT64_C(65535);
    TIM3->CNT=(uint16_t)sync_time-1000U;
    IMU_SyncPoll(); /* empty FIFO observed recently despite counter wrap */
    command(2,77,2);assert(sync_ambiguous && pending && lease_ms==lease);
    command(3,77,3);assert(!active && !IMU_SyncExtend(data));
    command(1,77,2);assert(!active); /* delayed START cannot resurrect STOP */
    command(2,77,4);assert(!active);
    command(1,88,0xfffffffeU);command(2,88,0xffffffffU);command(2,88,0);assert(active && sequence==0);
    tick+=1000;IMU_SyncPoll();assert(!active && !IMU_SyncExtend(data));
    IMU_SyncSourceSample(41);uint32_t old=epoch;
    tick+=60;IMU_SyncSourceSample(42);assert(epoch!=old && source_time==42);
    timer_high=65536;TIM3->CNT=4;TIM3->SR=TIM_SR_UIF;assert(timer_now()==131076);
    TIM3->SR=0;assert(timer_now()==65540);
    timer_high=0;TIM3->CNT=1000;DWT->CYCCNT=100000;IMU_SyncTimerIRQ();
    /* Hardware cycle counter still detects a lost TIM3 wrap if HAL tick stops. */
    TIM3->CNT=(141000U & 65535U);TIM3->SR=TIM_SR_UIF;
    DWT->CYCCNT+=14000000;IMU_SyncTimerIRQ();assert(clock_fault);
    clock_fault=false;TIM3->SR=0;
    command(1,99,1);clock_fault=true;assert(!IMU_SyncExtend(data));IMU_SyncPoll();assert(!active);
    puts("native sync codec/state/rollover: PASS");return 0;
}
