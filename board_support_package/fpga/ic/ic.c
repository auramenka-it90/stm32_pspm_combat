/**
 ******************************************************************************
 * @file    ic.c
 * @brief   Interrupt Controller & Raw DMA Telemetry Dispatcher Implementation.
 *          Manages 400 Hz EXTI10 hardware events, thread notifications,
 *          high-speed 34-word SPI DMA acquisition, and DAC output updates.
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2).
 *          All comments in pure ASCII English.
 ******************************************************************************
 */

#include "ic.h"
#include <string.h>

/* ========================================================================= */
/*  PRIVATE DRIVER CONTEXT & STORAGE                                         */
/* ========================================================================= */

static FPGA_HandleTypeDef     *g_hbridge           = NULL;
static FpgaDmaTelemetryPacket_t g_raw_telemetry    = {0};

uint16_t                       g_fpga_status_word  = 0U;   /* Last sticky status image (Word 33) */
static uint16_t                g_prev_status_word  = 0U;
uint32_t                       g_fpga_st_evt[IC_ST_EVT_COUNT] = {0}; /* Rising-edge event counters */

static osThreadId_t            g_worker_task       = NULL;
static uint32_t                g_event_flag        = IC_EVENT_FLAG_FPGA_IRQ;

static osMutexId_t             g_ic_mutex          = NULL;

static volatile uint32_t       g_last_irq_tick     = 0;
static volatile uint32_t       g_frame_counter     = 0;
static volatile bool           g_is_400hz_valid    = false;
static volatile bool           g_is_fpga_ok        = true;

/* ========================================================================= */
/*  HARDWARE EXTI CALLBACK (Interrupt Context)                               */
/* ========================================================================= */

/**
 * @brief  HAL GPIO External Interrupt Callback.
 * @note   Triggered on falling edge of INTERRUPT_FPGA_Pin (PB10 / EXTI10).
 *         Records system tick timestamp and wakes the FreeRTOS sequencer task.
 * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == INTERRUPT_FPGA_Pin){
        g_last_irq_tick = osKernelGetTickCount();
        if(g_worker_task != NULL){
            osThreadFlagsSet(g_worker_task, g_event_flag);
        }
    }
}

/* ========================================================================= */
/*  PUBLIC INITIALIZATION & TASK BINDING APIS                                */
/* ========================================================================= */

/**
 * @brief  Initializes the Interrupt Controller dispatcher and allocates mutex.
 * @param  hbridge: Pointer to initialized FPGA SPI Bridge handle.
 * @param  worker_task: Handle of the FreeRTOS thread to be notified (or NULL).
 * @param  event_flag: 32-bit OS flag mask used for task wake-up.
 * @retval true if initialization succeeded and mutex was created, false otherwise.
 */
bool IC_Init(FPGA_HandleTypeDef *hbridge, osThreadId_t worker_task, uint32_t event_flag){
    if(!hbridge) return false;

    g_hbridge        = hbridge;
    g_worker_task    = worker_task;
    g_event_flag     = (event_flag != 0) ? event_flag : IC_EVENT_FLAG_FPGA_IRQ;
    g_frame_counter  = 0;
    g_last_irq_tick  = 0;
    g_is_400hz_valid = false;
    g_is_fpga_ok     = true;

    memset(&g_raw_telemetry, 0, sizeof(FpgaDmaTelemetryPacket_t));

    const osMutexAttr_t mutex_attr = {
        .name      = "IC_Mutex",
        .attr_bits = osMutexPrioInherit
    };
    g_ic_mutex = osMutexNew(&mutex_attr);
    return (g_ic_mutex != NULL);
}

/**
 * @brief  Binds or rebinds the target FreeRTOS worker thread for FPGA IRQ events.
 * @param  worker_task: Handle of the target task.
 * @param  event_flag: 32-bit notification event flag.
 * @retval None
 */
void IC_AttachTask(osThreadId_t worker_task, uint32_t event_flag){
    g_worker_task = worker_task;
    g_event_flag  = (event_flag != 0) ? event_flag : IC_EVENT_FLAG_FPGA_IRQ;
}

/* ========================================================================= */
/*  REAL-TIME 400 Hz EXECUTION ENGINE                                        */
/* ========================================================================= */

/**
 * @brief  Executes one iteration of the 400 Hz synchronous real-time loop.
 * @note   1. Waits for EXTI10 notification flag.
 *         2. Reads 34-word RAW telemetry burst via SPI1 DMA (NSS_D / PB2).
 *         3. Validates 400 Hz carrier frequency limits (375..425 Hz).
 *         4. Copies data thread-safely into internal telemetry buffer.
 * @param  timeout_ms: Maximum wait timeout in milliseconds.
 * @retval true if frame was successfully acquired and carrier is valid, false on timeout/fault.
 */
bool IC_Process(uint32_t timeout_ms){
    FpgaDmaTelemetryPacket_t local_rx;

    if(!g_hbridge) return false;

    // 1. Wait for FPGA IRQ notification from ISR (data-ready or freq-error,
    //    both lines are OR'd onto PB10 and set the same event flag).
    //    osThreadFlagsWait() returns a NEGATIVE osFlagsError* code on timeout
    //    or error; those bits could otherwise alias into g_event_flag and be
    //    mistaken for a real event, so reject any negative status first.
    uint32_t flags = osThreadFlagsWait(g_event_flag, osFlagsWaitAny, timeout_ms);
    if(((int32_t)flags < 0) || ((flags & g_event_flag) == 0U)){
        g_is_400hz_valid = false;
        return false;
    }

    // 2. Fetch 34-word telemetry burst from FPGA over high-speed SPI1 DMA
    if(FPGA_Read_Telemetry_DMA(g_hbridge, &local_rx, 10) != FPGA_OK){
        g_is_fpga_ok     = false;
        g_is_400hz_valid = false;
        return false;
    }
    g_is_fpga_ok = true;

    // 2a. Release the edge-latched PENDING bits in the FPGA interrupt
    //     controller (W1C). The data-ready source flag was already cleared
    //     automatically by the DMA burst (dma_ack), but the controller latch
    //     is CPU-only: without this write irq_out_N stays asserted and PB10
    //     never produces another falling edge (one IRQ per power cycle).
    //     Persistent freq-error stays a level high, so no new rising edge
    //     occurs until the fault recovers - no interrupt storm is possible.
    (void)IC_FPGA_ClearPending(FPGA_IRQ_LINES_ALL_MASK);

    // 3. Hardware Carrier Frequency Watchdog Check (375..425 Hz Corridor: 23529..26666 ticks)
    if(local_rx.period_uon12 < FPGA_COMP_DEF_MIN_PER_TICKS ||
       local_rx.period_uon12 > FPGA_COMP_DEF_MAX_PER_TICKS ||
       local_rx.period_uon23 < FPGA_COMP_DEF_MIN_PER_TICKS ||
       local_rx.period_uon23 > FPGA_COMP_DEF_MAX_PER_TICKS){
        g_is_400hz_valid = false;
    } else {
        g_is_400hz_valid = true;
    }

    // 4. Thread-safe atomic update of the global telemetry buffer
    //    + rising-edge event counters for FPGA sticky flags
    osMutexAcquire(g_ic_mutex, osWaitForever);
    {
        uint16_t sw       = local_rx.status_word;
        uint16_t new_bits = (uint16_t)(sw & (uint16_t)~g_prev_status_word);

        if(new_bits & FPGA_ST_ADC_ERR) g_fpga_st_evt[IC_ST_EVT_ADC_ERR]++;
        if(new_bits & FPGA_ST_RES_ERR) g_fpga_st_evt[IC_ST_EVT_RES_ERR]++;
        if(new_bits & FPGA_ST_RES_OVR) g_fpga_st_evt[IC_ST_EVT_RES_OVR]++;
        if(new_bits & FPGA_ST_ERR_12)  g_fpga_st_evt[IC_ST_EVT_ERR_12]++;
        if(new_bits & FPGA_ST_ERR_23)  g_fpga_st_evt[IC_ST_EVT_ERR_23]++;

        g_prev_status_word = sw;
        g_fpga_status_word = sw;
    }
    g_raw_telemetry = local_rx;
    g_frame_counter++;
    osMutexRelease(g_ic_mutex);

    // 5. Release the comparator frequency-error sticky latch (CTRL B15
    //    IRQ_ACK). Unlike the ADC/resolver stickies, which the FPGA itself
    //    clears on every DMA burst (dma_ack), the comparator err_12/err_23
    //    latches are CPU-only: without this write the flags keep haunting
    //    status word 33, the rising-edge event counters freeze at their
    //    first value and GLOBAL_ERROR in the host packet stays pinned
    //    until a power cycle. Sent on every frame while the bits are set:
    //    the counters still count episodes, not periods, because the edge
    //    detector works on transitions, and EN_MEAS is re-asserted with
    //    its current value (a 1->1 write does not restart the FSM).
    if(local_rx.status_word & (FPGA_ST_ERR_12 | FPGA_ST_ERR_23)){
        (void)FPGA_Write_Poll(g_hbridge, ADDR_S_COMP_CTRL,
                              FPGA_COMP_CTRL_IRQ_ACK | FPGA_COMP_CTRL_IRQ_EN |
                              FPGA_COMP_CTRL_EN_MEAS, 100);
    }

    return g_is_400hz_valid;
}

/* ========================================================================= */
/*  THREAD-SAFE DATA ACCESS & CONTROL APIS                                   */
/* ========================================================================= */

/**
 * @brief  Fetches a local copy of the latest 34-word RAW telemetry packet.
 * @param  dest: Pointer to destination structure.
 * @retval true if data is valid and fresh, false if carrier is in fault.
 */
bool IC_GetRawTelemetry(FpgaDmaTelemetryPacket_t *dest){
    if(!dest || !g_ic_mutex) return false;

    osMutexAcquire(g_ic_mutex, osWaitForever);
    *dest = g_raw_telemetry;
    osMutexRelease(g_ic_mutex);

    return g_is_400hz_valid;
}

/**
 * @brief  Transmits 10-channel DAC setpoints directly to FPGA via SPI1 DMA.
 * @param  dac_data: Pointer to 10-word DAC setpoints structure.
 * @param  timeout_ms: Maximum transfer timeout in milliseconds.
 * @retval FPGA_OK if transfer succeeded, error code otherwise.
 */
FPGA_Status_t IC_WriteDacSetpoints(const FpgaDmaDacPacket_t *dac_data, uint32_t timeout_ms){
    if(!g_hbridge || !dac_data) return FPGA_ERROR;
    return FPGA_Write_DAC_DMA(g_hbridge, (const FpgaDmaDacPacket_t*)dac_data, timeout_ms);
}

/**
 * @brief  Checks whether 400 Hz carrier interrupts are actively arriving.
 * @retval true if last interrupt occurred within IC_400HZ_TIMEOUT_MS, false otherwise.
 */
bool IC_Is400HzValid(void){
    if((osKernelGetTickCount() - g_last_irq_tick) > IC_400HZ_TIMEOUT_MS){
        g_is_400hz_valid = false;
    }
    return g_is_400hz_valid;
}

/**
 * @brief  Assembles the 8-bit hardware system health status byte for the Host packet.
 * @note   Bit 0 = Global Error (any fault: telemetry unreliable, BC decides itself),
 *         Bit 1 = ADC Error (FPGA sticky ADC/RES watchdog/overrun flags),
 *         Bit 2 = FPGA Error (SPI link / POST failure only).
 *         Bit 3 (Pressure Error) is NOT set here: the host layer ORs it in
 *         from the BMP280 validity gate when assembling the packet.
 * @retval Formatted 8-bit status byte (bits 0..2 only).
 */
uint8_t IC_GetSystemStatusByte(void){
    uint8_t status = IC_STATUS_OK;
    uint16_t sw    = IC_GetStatusWord();

    if(!IC_Is400HzValid() || !g_is_fpga_ok ||
       ((sw & FPGA_ST_ALL_MASK) != 0U)){
        status |= IC_STATUS_GLOBAL_ERROR;
    }
    if((sw & (FPGA_ST_ADC_ERR | FPGA_ST_RES_ERR | FPGA_ST_RES_OVR)) != 0U){
        status |= IC_STATUS_ADC_ERROR;
    }
    if(!g_is_fpga_ok){
        status |= IC_STATUS_FPGA_ERROR;
    }

    return status;
}

/**
 * @brief  Returns the latest FPGA sticky status word (Burst Word 33, FPGA_ST_* bits).
 * @retval 16-bit sticky flag image from the last 400 Hz snapshot.
 */
uint16_t IC_GetStatusWord(void){
    uint16_t sw;

    if(!g_ic_mutex) return 0U;
    osMutexAcquire(g_ic_mutex, osWaitForever);
    sw = g_fpga_status_word;
    osMutexRelease(g_ic_mutex);
    return sw;
}

/**
 * @brief  Returns sticky-flag rising-edge event counter (since boot or last reset).
 * @param  idx: Event index IC_ST_EVT_* (0..IC_ST_EVT_COUNT-1).
 * @retval 32-bit event count, 0 on invalid index.
 */
uint32_t IC_GetStatusEvtCount(uint8_t idx){
    if(idx >= (uint8_t)IC_ST_EVT_COUNT) return 0U;
    return g_fpga_st_evt[idx];
}

/**
 * @brief  Clears all FPGA sticky-flag event counters.
 * @retval None
 */
void IC_ResetStatusEvtCounts(void){
    uint8_t i;

    if(!g_ic_mutex) return;
    osMutexAcquire(g_ic_mutex, osWaitForever);
    for(i = 0U; i < (uint8_t)IC_ST_EVT_COUNT; i++){
        g_fpga_st_evt[i] = 0U;
    }
    osMutexRelease(g_ic_mutex);
}

/**
 * @brief  Returns total count of successfully processed 400 Hz telemetry frames.
 * @retval 32-bit frame counter.
 */
uint32_t IC_GetFrameCount(void){
    return g_frame_counter;
}

/* ========================================================================= */
/*  FPGA DEVICE ID 2: LOW-LEVEL POLLING CONTROL                              */
/* ========================================================================= */

/**
 * @brief  Configures hardware Interrupt Controller registers inside FPGA (Device ID = 2).
 * @param  mask: 16-bit Interrupt Mask (1 = Enabled, 0 = Disabled).
 * @param  edge_sel: 16-bit Edge polarity (0 = Rising edge, 1 = Falling edge).
 * @param  global_en: Master Global Interrupt Enable (GIE) flag.
 * @retval FPGA_OK on success, error code otherwise.
 */
FPGA_Status_t IC_FPGA_Configure(uint16_t mask, uint16_t edge_sel, bool global_en){
    FPGA_Status_t status;
    uint16_t ctrl_val = global_en ? FPGA_INT_CTRL_GIE_ENABLE : 0x0000U;

    if(!g_hbridge) return FPGA_ERROR;

    status = FPGA_Write_Poll(g_hbridge, ADDR_S_INT_MASK, mask, 100);
    if(status != FPGA_OK) return status;

    status = FPGA_Write_Poll(g_hbridge, ADDR_S_INT_EDGE_SEL, edge_sel, 100);
    if(status != FPGA_OK) return status;

    return FPGA_Write_Poll(g_hbridge, ADDR_S_INT_CTRL, ctrl_val, 100);
}

/**
 * @brief  Reads active masked pending interrupt flags from FPGA (Device ID = 2).
 * @param  pending: Pointer to store 16-bit masked pending flags.
 * @retval FPGA_OK on success, error code otherwise.
 */
FPGA_Status_t IC_FPGA_GetPending(uint16_t *pending){
    if(!g_hbridge || !pending) return FPGA_ERROR;
    return FPGA_Read_Poll(g_hbridge, ADDR_S_INT_PENDING, pending, 100);
}

/**
 * @brief  Clears pending interrupt flags in FPGA using Write-1-to-Clear (W1C).
 * @param  clear_mask: Bitmask of flags to be cleared.
 * @retval FPGA_OK on success, error code otherwise.
 */
FPGA_Status_t IC_FPGA_ClearPending(uint16_t clear_mask){
    if(!g_hbridge) return FPGA_ERROR;
    return FPGA_Write_Poll(g_hbridge, ADDR_S_INT_PENDING, clear_mask, 100);
}
