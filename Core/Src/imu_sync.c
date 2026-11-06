/* Optional CAN-only clock anchors. No clock steering; no floating point in ISR. */
#include "imu_sync.h"
#if OPENIMU_CAN_SYNC
#include <string.h>
/* Register access seams allow native tests to model rc_w0/read-to-clear effects. */
#ifndef IMU_SYNC_TIM_STATUS
#define IMU_SYNC_TIM_STATUS() (TIM3->SR)
#endif
#ifndef IMU_SYNC_TIM_COUNTER
#define IMU_SYNC_TIM_COUNTER() (TIM3->CNT)
#endif
#ifndef IMU_SYNC_TIM_CAPTURE
#define IMU_SYNC_TIM_CAPTURE() (TIM3->CCR3)
#endif
#ifndef IMU_SYNC_TIM_CLEAR
#define IMU_SYNC_TIM_CLEAR(flags) (TIM3->SR = ~(flags))
#endif
#ifndef IMU_SYNC_RX_CLEAR_LOST
#define IMU_SYNC_RX_CLEAR_LOST() (bus->Instance->IR = FDCAN_IR_RF0L)
#endif
static FDCAN_HandleTypeDef *bus;
static volatile uint64_t timer_high, capture_time;
static volatile uint32_t capture_seq, overrun_count;
static volatile bool clock_fault;
static uint32_t epoch, session, sequence, lease_ms, last_poll_ms, invalid_count;
static uint64_t source_time, sync_time, last_rx_empty;
static uint32_t irq_last_cycles, consumed_capture_seq, last_overrun;
static uint64_t irq_last_time;
static bool irq_seen;
static uint32_t reported_epoch, revocation_epoch;
static uint16_t source_last;
static uint32_t source_ms;
static bool ready, active, pending, source_valid, sync_ambiguous, rx_tainted;
static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static void put32(uint8_t *p, uint32_t n)
{
    p[0] = (uint8_t)(n >> 24);
    p[1] = (uint8_t)(n >> 16);
    p[2] = (uint8_t)(n >> 8);
    p[3] = (uint8_t)n;
}
static void put64(uint8_t *p, uint64_t n)
{
    put32(p, (uint32_t)(n >> 32));
    put32(p + 4, (uint32_t)n);
}
/* Caller holds PRIMASK or is the sole TIM3 ISR. Account for a pending overflow. */
static uint64_t timer_now(void)
{
    uint64_t high = timer_high;
    uint32_t low = IMU_SYNC_TIM_COUNTER();
    if ((IMU_SYNC_TIM_STATUS() & TIM_SR_UIF) != 0U) {
        high += 65536U;
        low = IMU_SYNC_TIM_COUNTER();
    }
    return high + low;
}
static uint64_t now_atomic(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint64_t t = timer_now();
    __set_PRIMASK(mask);
    return t;
}
void IMU_SyncTimerIRQ(void)
{
    uint32_t flags = IMU_SYNC_TIM_STATUS();
    /* Never read CCR3 without CC3IF: a new edge between SR and CCR could otherwise be acknowledged
     * and lost. */
    uint16_t captured = (flags & TIM_SR_CC3IF) != 0U ? (uint16_t)IMU_SYNC_TIM_CAPTURE() : 0U;
    uint64_t now = timer_now();
    uint32_t cycles = DWT->CYCCNT;
    bool capture_fresh = true;
    if (irq_seen) {
        uint64_t wall = (uint32_t)(cycles - irq_last_cycles) / (SystemCoreClock / 1000000U);
        uint64_t elapsed = now - irq_last_time;
        /* CCR3 has only 16 bits. Even a correctly extended timer cannot date
         * an unread capture older than one wrap. Keep a conservative margin
         * below 65.536 ms; UIF-only interrupts do not invalidate the clock. */
        capture_fresh = wall < 60000U && elapsed < 60000U;
        if (wall > elapsed + 5000U || elapsed > wall + 5000U) {
            clock_fault = true;
        }
    }
    irq_seen = true;
    irq_last_cycles = cycles;
    irq_last_time = now;
    if ((flags & TIM_SR_UIF) != 0U) {
        timer_high += 65536U;
        IMU_SYNC_TIM_CLEAR(TIM_SR_UIF);
    }
    if ((flags & TIM_SR_CC3OF) != 0U) {
        overrun_count++;
        IMU_SYNC_TIM_CLEAR(TIM_SR_CC3OF);
    }
    if ((flags & TIM_SR_CC3IF) != 0U) {
        if (capture_fresh) {
            uint16_t age = (uint16_t)((uint16_t)now - captured);
            capture_time = now - age;
            capture_seq++;
        } else {
            overrun_count++;
        }
        /* Reading CCR3 after SR acknowledges CC3IF in either case. */
    }
}
static bool random_epoch(void)
{
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    uint32_t start = HAL_GetTick();
    while ((RCC->CRRCR & RCC_CRRCR_HSI48RDY) == 0U) {
        if (HAL_GetTick() - start > 10U) {
            return false;
        }
    }
    __HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_HSI48);
    __HAL_RCC_RNG_CLK_ENABLE();
    RNG->CR = RNG_CR_RNGEN;
    start = HAL_GetTick();
    while ((RNG->SR & RNG_SR_DRDY) == 0U) {
        if (HAL_GetTick() - start > 10U) {
            RNG->CR = 0U;
            return false;
        }
    }
    if ((RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0U) {
        RNG->CR = 0U;
        return false;
    }
    epoch = RNG->DR;
    RNG->CR = 0U;
    return epoch != 0U;
}
bool IMU_SyncInit(FDCAN_HandleTypeDef *can)
{
    ready = active = pending = source_valid = sync_ambiguous = rx_tainted = false;
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->DIER = 0U;
    TIM3->CR1 = 0U;
    TIM3->SR = 0U;
    HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
    timer_high = capture_time = source_time = sync_time = last_rx_empty = 0U;
    capture_seq = overrun_count = consumed_capture_seq = last_overrun = 0U;
    irq_seen = clock_fault = false;
    irq_last_cycles = 0U;
    irq_last_time = 0U;
    epoch = session = sequence = lease_ms = last_poll_ms = invalid_count = 0U;
    reported_epoch = revocation_epoch = source_ms = 0U;
    source_last = 0U;
    bus = can;
    if (can == NULL || can->Instance == NULL) {
        return false;
    }
    if (!random_epoch()) {
        return false;
    }
    __HAL_RCC_TIM3_CLK_ENABLE();
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) {
        clock *= 2U;
    }
    if (clock < 1000000U || clock % 1000000U != 0U || SystemCoreClock < 1000000U ||
        SystemCoreClock % 1000000U != 0U) {
        return false;
    }
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x700U;
    filter.FilterID2 = 0x7FFU;
    if (HAL_FDCAN_ConfigFilter(can, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(can, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ConfigTimestampCounter(can, FDCAN_TIMESTAMP_PRESC_1) != HAL_OK ||
        HAL_FDCAN_EnableTimestampCounter(can, FDCAN_TIMESTAMP_EXTERNAL) != HAL_OK) {
        return false;
    }
    TIM3->CR1 = 0U;
    TIM3->PSC = clock / 1000000U - 1U;
    TIM3->ARR = 65535U;
    TIM3->CCMR2 = TIM_CCMR2_CC3S_0;
    TIM3->CCER = TIM_CCER_CC3E;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC3IE;
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    /* Existing EXTI routing also sees input in alternate-function mode. */
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_NVIC_SetPriority(TIM3_IRQn, 4U, 0U);
    HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    irq_last_cycles = DWT->CYCCNT;
    irq_last_time = 0U;
    irq_seen = true;
    TIM3->CR1 = TIM_CR1_CEN;
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    last_rx_empty = now_atomic();
    last_poll_ms = HAL_GetTick();
    ready = true;
    return true;
}
void IMU_SyncSourceInvalid(void)
{
    if (++epoch == 0U) {
        epoch = 1U;
    }
    source_valid = false;
    invalid_count++;
}
void IMU_SyncSourceSample(uint16_t stamp)
{
    uint32_t ms = HAL_GetTick();
    if (source_valid && ms - source_ms >= 60U) {
        IMU_SyncSourceInvalid();
    }
    if (!source_valid) {
        source_time = stamp;
        source_valid = true;
    } else {
        uint16_t delta = (uint16_t)(stamp - source_last);
        if (delta == 0U) {
            IMU_SyncSourceInvalid();
            source_time = stamp;
            source_valid = true;
        } else {
            source_time += delta;
        }
    }
    source_last = stamp;
    source_ms = ms;
}
/* Preserve the identity of an already published anchor while moving all future
 * samples to a new epoch. The DATA epoch is a second invalidation path if the
 * revocation itself is lost on the bus. */
static void mark_sync_ambiguous(void)
{
    if (!sync_ambiguous) {
        revocation_epoch = reported_epoch != 0U ? reported_epoch : epoch;
        IMU_SyncSourceInvalid();
    }
    sync_ambiguous = true;
    pending = true;
}
static bool observe_rx_loss(void)
{
    if ((bus->Instance->IR & FDCAN_IR_RF0L) == 0U) {
        return false;
    }
    IMU_SYNC_RX_CLEAR_LOST();
    rx_tainted = true;
    if (active) {
        mark_sync_ambiguous();
    }
    return true;
}
void IMU_SyncPoll(void)
{
    uint32_t ms = HAL_GetTick();
    bool timely = ms - last_poll_ms < 10U;
    last_poll_ms = ms;
    if (active && (ms - lease_ms >= 1000U || clock_fault)) {
        active = false;
        pending = false;
    }
    if (!ready) {
        return;
    }
    if (observe_rx_loss()) {
        timely = false;
    }
    for (unsigned i = 0; i < 3U && HAL_FDCAN_GetRxFifoFillLevel(bus, FDCAN_RX_FIFO0) > 0U; i++) {
        uint8_t data[64];
        FDCAN_RxHeaderTypeDef h = {0};
        if (HAL_FDCAN_GetRxMessage(bus, FDCAN_RX_FIFO0, &h, data) != HAL_OK) {
            break;
        }
        if (observe_rx_loss()) {
            timely = false;
        }
        if (!timely || rx_tainted || clock_fault || h.Identifier != 0x700U ||
            h.IdType != FDCAN_STANDARD_ID || h.RxFrameType != FDCAN_DATA_FRAME ||
            h.FDFormat != FDCAN_FD_CAN || h.DataLength != FDCAN_DLC_BYTES_16 ||
            memcmp(data, "OSYN", 4) != 0 || data[4] != 1U || data[6] != 0U || data[7] != 0U) {
            continue;
        }
        uint32_t s = be32(data + 8), q = be32(data + 12);
        uint8_t op = data[5];
        if (s == 0U || op < 1U || op > 3U) {
            continue;
        }
        if (active && s != session) {
            continue;
        }
        uint64_t now = now_atomic();
        /* A successful empty observation bounds FIFO dwell even if the task
         * was preempted or earlier HAL reads failed. Check after every pop. */
        if (now - last_rx_empty >= 10000U) {
            continue;
        }
        uint16_t age = (uint16_t)((uint16_t)now - (uint16_t)h.RxTimestamp);
        if (age > 10000U) {
            continue;
        }
        uint64_t received = now - age;
        if (active && s == session && q == sequence && op != 3U) {
            /* A retransmitted sequence can represent a different physical SOF.
             * Revoke this anchor without renewing its lease, including after an
             * earlier report was queued. The host invalidates the whole group. */
            if (received != sync_time) {
                mark_sync_ambiguous();
            }
            continue;
        }
        if (s == session && (int32_t)(q - sequence) <= 0) {
            continue;
        }
        if (op == 3U) {
            if (active && s == session) {
                active = false;
                pending = false;
                sequence = q;
            }
            continue;
        }
        if (!active && op != 1U) {
            continue;
        }
        if (sync_ambiguous && pending) {
            continue; /* Do not overwrite a revocation still waiting for TX room. */
        }
        reported_epoch = 0U;
        session = s;
        sequence = q;
        lease_ms = ms;
        active = true;
        pending = true;
        sync_ambiguous = false;
        sync_time = received;
    }
    uint64_t empty_check_time = now_atomic();
    if (HAL_FDCAN_GetRxFifoFillLevel(bus, FDCAN_RX_FIFO0) == 0U) {
        last_rx_empty = empty_check_time;
        rx_tainted = false;
    }
}
uint32_t IMU_SyncCaptureToken(void)
{
    uint32_t token = capture_seq;
    if (token == consumed_capture_seq) {
        return 0U;
    }
    consumed_capture_seq = token;
    return token;
}
void IMU_SyncReport(uint32_t token, bool clean)
{
    if (!active || !ready || clock_fault || HAL_GetTick() - lease_ms >= 1000U) {
        return;
    }
    (void)observe_rx_loss();
    if (!pending) {
        return;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint64_t captured = capture_time;
    uint32_t current = capture_seq;
    bool intact = (IMU_SYNC_TIM_STATUS() & (TIM_SR_CC3IF | TIM_SR_CC3OF)) == 0U;
    bool clock_ok = !clock_fault;
    __set_PRIMASK(mask);
    if (!clock_ok) {
        return;
    }
    bool no_overrun = last_overrun == overrun_count;
    last_overrun = overrun_count;
    bool pair = !sync_ambiguous && clean && intact && no_overrun && source_valid &&
                token == current && current != 0U && captured >= sync_time;
    uint8_t data[48] = {0};
    data[0] = 1U;
    data[1] = sync_ambiguous ? 0U : (uint8_t)(1U | (pair ? 2U : 0U));
    data[2] = OPENIMU_NODE_ID;
    put32(data + 4, session);
    put32(data + 8, sequence);
    put32(data + 12, sync_ambiguous ? revocation_epoch : epoch);
    put64(data + 16, sync_time);
    if (pair) {
        put64(data + 24, captured);
        put64(data + 32, source_time);
    } else {
        invalid_count++;
    }
    put32(data + 40, overrun_count);
    put32(data + 44, invalid_count);
    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier = 0x710U + OPENIMU_NODE_ID;
    h.IdType = FDCAN_STANDARD_ID;
    h.TxFrameType = FDCAN_DATA_FRAME;
    h.DataLength = FDCAN_DLC_BYTES_48;
    h.BitRateSwitch = FDCAN_BRS_ON;
    h.FDFormat = FDCAN_FD_CAN;
    bool queued = HAL_FDCAN_GetTxFifoFreeLevel(bus) >= 2U &&
                  HAL_FDCAN_AddMessageToTxFifoQ(bus, &h, data) == HAL_OK;
    if (queued && !sync_ambiguous) {
        reported_epoch = epoch;
    }
    /* Ordinary anchors may be skipped. A revocation stays bounded by the
     * existing lease and prevents a new control from replacing it prematurely. */
    pending = sync_ambiguous && !queued;
}
bool IMU_SyncExtend(uint8_t *data)
{
    if (!active || !source_valid || clock_fault || HAL_GetTick() - lease_ms >= 1000U) {
        return false;
    }
    put32(data + 16, (uint32_t)source_time);
    put32(data + 20, epoch);
    return true;
}
#endif
