/* Native mock transport: compiles the real driver, not a duplicate parser. */
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#define __MAIN_H
#define HAL_MAX_DELAY UINT32_MAX
#define GPIOA 1
#define GPIO_PIN_4 16U
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
static HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
static void HAL_GPIO_WritePin(int port, unsigned pin, int level) { (void)port; (void)pin; (void)level; }
static void HAL_Delay(uint32_t ms) { (void)ms; }
#include "../../Core/Src/icm45686.c"
static uint8_t regs[128], mregs[65536], packet[16];
static uint16_t indirect;
static int fail_reg, fail_mreg, corrupt_reg, flush_stuck, count_reads, change_count;
static SPI_HandleTypeDef spi;
static HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t timeout)
{
    (void)h; (void)timeout;
    unsigned reg = tx[0] & 0x7fU;
    memset(rx, 0, n);
    if ((int)reg == fail_reg) return HAL_ERROR;
    if (tx[0] & 0x80U) {
        if (reg == ICM45686_REG_FIFO_DATA) { memcpy(rx + 1, packet, n - 1U); return HAL_OK; }
        if (reg == ICM45686_REG_IREG_DATA) {
            if ((int)indirect == fail_mreg) return HAL_ERROR;
            rx[1] = mregs[indirect++]; return HAL_OK;
        }
        if (reg == ICM45686_REG_FIFO_COUNT_0 && change_count) {
            ++count_reads; rx[1] = count_reads == 1 ? 9 : 3; return HAL_OK;
        }
        if (reg == ICM45686_REG_FIFO_CONFIG2 && !flush_stuck) regs[reg] &= 0x7fU;
        for (unsigned i = 1; i < n; ++i) rx[i] = regs[(reg+i-1U)&0x7fU];
        if ((int)reg == corrupt_reg) rx[1] ^= 0x40U;
    } else {
        for (unsigned i = 1; i < n; ++i) {
            unsigned a = (reg+i-1U)&0x7fU;
            regs[a] = tx[i];
            if (a == ICM45686_REG_IREG_ADDR_15_8) indirect = (uint16_t)((indirect&255U)|((uint16_t)tx[i]<<8));
            if (a == ICM45686_REG_IREG_ADDR_7_0) indirect = (uint16_t)((indirect&0xff00U)|tx[i]);
            if (a == ICM45686_REG_IREG_DATA) mregs[indirect++] = tx[i];
        }
    }
    return HAL_OK;
}
static void reset_mock(void)
{
    memset(regs,0,sizeof(regs)); memset(mregs,0,sizeof(mregs)); memset(packet,0,sizeof(packet));
    indirect=0; fail_reg=fail_mreg=corrupt_reg=-1; flush_stuck=count_reads=change_count=0;
    regs[ICM45686_REG_WHO_AM_I]=ICM45686_WHO_AM_I_VALUE;
    icm_spi=&spi; icm_endian_big=0; icm45686_frame_errors=0;
    packet[0]=0x68; for (unsigned i=0;i<6;++i) packet[1+2*i]=(uint8_t)(i+1);
    packet[13]=0xfe; packet[14]=0xfe; packet[15]=0xff;
}
static void packet_checks(void)
{
    ICM45686_Data_t d; uint16_t stamp;
    reset_mock(); packet[0]=0x7f; assert(ICM45686_ReadFifoPacket(&d,&stamp)==HAL_ERROR);
    for (unsigned h=0x68;h<=0x6b;++h) { packet[0]=(uint8_t)h; assert(ICM45686_ReadFifoPacket(&d,&stamp)==HAL_OK); }
    assert(d.accel[0]==1 && d.gyro[2]==6 && d.temp==-512 && stamp==65534);
    const uint8_t invalid[]={0,0x60,0x6c,0x78,0xe8,0x7f};
    for (unsigned i=0;i<sizeof(invalid);++i) { packet[0]=invalid[i]; assert(ICM45686_ReadFifoPacket(&d,&stamp)==HAL_ERROR); }
    packet[0]=0x68;
    for (unsigned i=1;i<13;i+=2) {
        uint8_t a=packet[i],b=packet[i+1]; packet[i]=0;packet[i+1]=0x80;
        assert(ICM45686_ReadFifoPacket(&d,&stamp)==HAL_ERROR); packet[i]=a;packet[i+1]=b;
    }
    uint32_t errors=ICM45686_GetFrameErrors();fail_reg=ICM45686_REG_FIFO_DATA;
    assert(ICM45686_ReadFifoPacket(&d,&stamp)==HAL_ERROR && ICM45686_GetFrameErrors()==errors);
    puts("PASS invalid-first recovery, dynamic ODR flags, format rejection, all six invalid axes, transport distinction");
}
static void configuration_checks(void)
{
    reset_mock(); assert(ICM45686_ConfigFifo()==HAL_OK);
    assert(regs[ICM45686_REG_FIFO_CONFIG0]==0x5e && regs[ICM45686_REG_FIFO_CONFIG3]==7);
    assert(regs[ICM45686_REG_FIFO_CONFIG1_0]==1 && regs[ICM45686_REG_FIFO_CONFIG4]==2);
    reset_mock();flush_stuck=1;assert(ICM45686_ConfigFifo()==HAL_TIMEOUT);
    reset_mock();fail_mreg=ICM45686_MREG_SREG_CTRL;assert(ICM45686_ConfigFifo()==HAL_ERROR);
    reset_mock();fail_reg=ICM45686_REG_FSYNC_CONFIG0;assert(ICM45686_ConfigFifo()==HAL_ERROR);
    reset_mock();corrupt_reg=ICM45686_REG_FIFO_CONFIG3;assert(ICM45686_ConfigFifo()==HAL_ERROR);
    puts("PASS depth, layout, watermark, flush timeout, endian/FSYNC/readback failure propagation");
}
static void count_checks(void)
{
    uint16_t n=999;
    reset_mock();assert(ICM45686_ReadFifoCountChecked(&n)==HAL_OK && n==0);
    change_count=1;assert(ICM45686_ReadFifoCountChecked(&n)==HAL_OK && n==3 && count_reads==2);
    fail_reg=ICM45686_REG_FIFO_COUNT_0;n=999;
    assert(ICM45686_ReadFifoCountChecked(&n)==HAL_ERROR && n==0);
    assert(ICM45686_ReadFifoCountChecked(NULL)==HAL_ERROR);
    reset_mock();mregs[ICM45686_MREG_SREG_CTRL]=2;assert(ICM45686_ConfigFifo()==HAL_OK);
    regs[ICM45686_REG_FIFO_COUNT_0]=1;regs[ICM45686_REG_FIFO_COUNT_1]=2;
    assert(ICM45686_ReadFifoCountChecked(&n)==HAL_OK && n==258);
    memset(packet+1,0,12);packet[1]=0x80;assert(ICM45686_ReadFifoPacket(&(ICM45686_Data_t){0},&n)==HAL_ERROR);
    puts("PASS empty FIFO, double-read erratum, read failure, null argument, shared big endian");
}
int main(void) { packet_checks();configuration_checks();count_checks();return 0; }
