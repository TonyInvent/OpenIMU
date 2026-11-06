#ifndef SYNC_MOCK_H
#define SYNC_MOCK_H
/* Actual firmware module with host HAL stubs. Register side effects/IRQ timing
 * require hardware validation; these tests cover codec, state and arithmetic. */
#define __MAIN_H
#define OPENIMU_CAN_SYNC 1
#define OPENIMU_NODE_ID 1
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
typedef struct { volatile uint32_t CNT, SR, CCR3, CR1, PSC, ARR, CCMR2, CCER, EGR, DIER; } Timer;
typedef struct { volatile uint32_t CRRCR, CFGR; } Rcc;
typedef struct { volatile uint32_t CR, SR, DR; } Rng;
typedef struct { volatile uint32_t IR, PSR, CCCR; } CanRegs;
typedef struct { CanRegs *Instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Pin,Mode,Pull,Speed,Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t IdType,FilterIndex,FilterType,FilterConfig,FilterID1,FilterID2; } FDCAN_FilterTypeDef;
typedef struct { uint32_t Identifier,IdType,RxFrameType,FDFormat,DataLength,RxTimestamp; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t Identifier,IdType,TxFrameType,DataLength,BitRateSwitch,FDFormat,ErrorStateIndicator,TxEventFifoControl,MessageMarker; } FDCAN_TxHeaderTypeDef;
static Timer timer_regs;
static Rcc rcc_regs;
static Rng rng_regs;
static struct { uint32_t CTRL, CYCCNT; } dwt_regs;
static struct { uint32_t DEMCR; } debug_regs;
static uint32_t SystemCoreClock=100000000U;
#define DWT (&dwt_regs)
#define CoreDebug (&debug_regs)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define TIM3 (&timer_regs)
#define RCC (&rcc_regs)
#define RNG (&rng_regs)
#define GPIOB 0
#define HAL_OK 0
#define TIM_SR_UIF 1U
#define TIM_SR_CC3IF 8U
#define TIM_SR_CC3OF 2048U
#define RCC_CRRCR_HSI48ON 1U
#define RCC_CRRCR_HSI48RDY 2U
#define RNG_CR_RNGEN 4U
#define RNG_SR_DRDY 1U
#define RNG_SR_CECS 2U
#define RNG_SR_SECS 4U
#define RCC_CFGR_PPRE1 0x700U
#define TIM_CCMR2_CC3S_0 1U
#define TIM_CCER_CC3E 256U
#define TIM_EGR_UG 1U
#define TIM_DIER_UIE 1U
#define TIM_DIER_CC3IE 8U
#define TIM_CR1_CEN 1U
#define FDCAN_IR_RF0L 8U
#define GPIO_PIN_0 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF2_TIM3 2U
#define TIM3_IRQn 29U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_FILTER_MASK 0U
#define FDCAN_FILTER_TO_RXFIFO0 0U
#define FDCAN_REJECT 0U
#define FDCAN_REJECT_REMOTE 0U
#define FDCAN_TIMESTAMP_PRESC_1 0U
#define FDCAN_TIMESTAMP_EXTERNAL 2U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_FD_CAN 1U
#define FDCAN_BRS_ON 1U
#define FDCAN_DLC_BYTES_16 16U
#define FDCAN_DLC_BYTES_48 48U
#define RCC_RNGCLKSOURCE_HSI48 0U
#define __HAL_RCC_RNG_CONFIG(x) ((void)(x))
#define __HAL_RCC_RNG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
static uint32_t tick;
static uint32_t tick_step, pclk=100000000U;
static unsigned config_stage, config_failure, tx_free=3;
static int tx_result, rx_result;
static void (*rx_hook)(void);
static void (*register_hook)(char);
static uint32_t mock_status(void) {if(register_hook)register_hook('S');return TIM3->SR;}
static uint32_t mock_counter(void) {if(register_hook)register_hook('N');return TIM3->CNT;}
static uint32_t mock_capture(void) {if(register_hook)register_hook('C');uint32_t v=TIM3->CCR3;TIM3->SR&=~TIM_SR_CC3IF;return v;}
static void mock_clear(uint32_t flags) {if(register_hook)register_hook('W');TIM3->SR&=~flags;}
#define IMU_SYNC_TIM_STATUS() mock_status()
#define IMU_SYNC_TIM_COUNTER() mock_counter()
#define IMU_SYNC_TIM_CAPTURE() mock_capture()
#define IMU_SYNC_TIM_CLEAR(flags) mock_clear(flags)
#define IMU_SYNC_RX_CLEAR_LOST() (bus->Instance->IR&=~FDCAN_IR_RF0L)
static unsigned rx_count, tx_count;
static int (*tx_hook)(FDCAN_TxHeaderTypeDef *,uint8_t *);
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[64], tx_data[64];
static uint32_t HAL_GetTick(void) { uint32_t value=tick;tick+=tick_step;return value; }
static uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk; }
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t x) {(void)x;}
#define HAL_GPIO_Init(a,b) ((void)(a),(void)(b))
#define HAL_NVIC_SetPriority(a,b,c) ((void)(a),(void)(b),(void)(c))
#define HAL_NVIC_ClearPendingIRQ(a) ((void)(a))
#define HAL_NVIC_EnableIRQ(a) ((void)(a))
#define HAL_NVIC_DisableIRQ(a) ((void)(a))
static int mock_config(void) {return ++config_stage==config_failure ? 1 : HAL_OK;}
#define HAL_FDCAN_ConfigFilter(a,b) ((void)(a),(void)(b),mock_config())
#define HAL_FDCAN_ConfigGlobalFilter(a,b,c,d,e) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),mock_config())
#define HAL_FDCAN_ConfigTimestampCounter(a,b) ((void)(a),(void)(b),mock_config())
#define HAL_FDCAN_EnableTimestampCounter(a,b) ((void)(a),(void)(b),mock_config())
static uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *b,uint32_t f) {(void)b;(void)f;return rx_count;}
static int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *b,uint32_t f,FDCAN_RxHeaderTypeDef *h,uint8_t *d) {(void)b;(void)f;if(rx_hook)rx_hook();if(rx_result)return rx_result;*h=rx_header;memcpy(d,rx_data,64);rx_count=0;return HAL_OK;}
static unsigned HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *b) {(void)b;return tx_free;}
static int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *b,FDCAN_TxHeaderTypeDef *h,uint8_t *d) {(void)b;if(tx_hook)return tx_hook(h,d);assert(h->Identifier==0x711);if(tx_result)return tx_result;memcpy(tx_data,d,48);tx_count++;return HAL_OK;}

#endif
