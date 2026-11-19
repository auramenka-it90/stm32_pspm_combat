/**
 ******************************************************************************
 * @file    host.c
 * @brief   Implementation of PSPM <-> BC Serial Protocol Driver (RS-422, 921600).
 *          Features 32-Byte Packet Assembly, CRC-16-CCITT, Cumulative XOR,
 *          FSM Receiver Parser, and Asynchronous 10-Channel DAC Updating.
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2).
 *          All comments in pure ASCII English.
 ******************************************************************************
 */

#include "host.h"
#include <string.h>
#include "app_tasks.h"

/* ========================================================================= */
/*  PRIVATE TYPES & DRIVER CONTEXT                                           */
/* ========================================================================= */

/**
 * @brief FSM States for byte-by-byte incoming packet reconstruction
 */
typedef enum {
    RX_STATE_FIND_START = 0, /* Waiting for Header byte (0x55) */
    RX_STATE_COLLECT_DATA,   /* Collecting 28 bytes of DAC payload */
    RX_STATE_GET_CRC_L,      /* Capturing CRC16 LSB */
    RX_STATE_GET_CRC_H,      /* Capturing CRC16 MSB */
    RX_STATE_VERIFY_XOR      /* Verifying XOR and committing packet */
} Rx_ParserState_t;

/**
 * @brief Driver context structure containing RTOS synchronization primitives
 */
typedef struct {
    UART_HandleTypeDef *huart;
    osMutexId_t         tx_mutex;   /* Protects TX buffer from concurrent task access */
    osSemaphoreId_t     tx_sem;     /* Blocks transmitting thread until DMA TX finishes */
    osSemaphoreId_t     rx_sem;     /* Wakes the consumer task when bytes enter the ring */
} host_uart_context_t;

static host_uart_context_t host_ctx = {0};

/* Global statistics and connection trackers */
Host_Stats_t       host_stats          = {0};
volatile uint32_t  last_bc_packet_tick = 0;
volatile bool      is_bc_link_error    = true;

/* 4-byte aligned static DMA buffers */
static uint8_t host_rx_buf[HOST_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t host_tx_buf[HOST_TX_BUF_SIZE] __attribute__((aligned(4)));

/* Lock-free SPSC byte ring: UART ISR producer -> task consumer.
   Sized 1024 B = 32 packets: survives ~80 ms of consumer stall at
   400 Hz inbound traffic. Parsing and FPGA access must never run
   in ISR context (FreeRTOS restriction) - they happen in the task
   via host_process_rx_pending(). */
#define HOST_RX_RING_SIZE  1024U
static uint8_t             host_rx_ring[HOST_RX_RING_SIZE];
static volatile uint32_t   host_rx_wr = 0U;   /* moved by ISR only */
static volatile uint32_t   host_rx_rd = 0U;   /* moved by task only */

/* ========================================================================= */
/*  PRIVATE PROTOCOL CHECKSUM HELPERS                                        */
/* ========================================================================= */

/**
 * @brief  Computes CRC-16-CCITT checksum over a byte buffer.
 * @note   Polynomial: 0x1021 (X^16 + X^12 + X^5 + 1), Initial Value: 0xFFFF.
 * @param  data: Pointer to byte array.
 * @param  length: Number of bytes to process.
 * @retval 16-bit computed CRC value.
 */
static uint16_t compute_crc16(const uint8_t *data, uint16_t length){
    uint16_t crc = PSPM_CRC_INIT;

    for(uint16_t i = 0; i < length; i++){
        crc ^= ((uint16_t)data[i] << 8);
        for(uint8_t bit = 0; bit < 8; bit++){
            if(crc & 0x8000U){
                crc = (crc << 1) ^ PSPM_CRC_POLYNOMIAL;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  Computes cumulative 8-bit XOR checksum across all bytes.
 * @param  data: Pointer to byte array.
 * @param  length: Number of bytes to XOR.
 * @retval 8-bit cumulative XOR value.
 */
static uint8_t compute_xor(const uint8_t *data, uint16_t length){
    uint8_t xor_val = 0U;

    for(uint16_t i = 0; i < length; i++){
        xor_val ^= data[i];
    }
    return xor_val;
}

/**
 * @brief  Byte-by-byte incoming packet FSM parser (BC -> PSPM, Header 0x55).
 * @note   1. Matches 0x55 start byte.
 *         2. Reconstructs 28-byte data field.
 *         3. Validates CRC-16-CCITT (Bytes 0..28) and XOR_ALL (Bytes 0..30).
 *         4. On success: Extracts 10 DAC setpoints and immediately calls IC_WriteDacSetpoints().
 * @param  rx_byte: Incoming byte from DMA buffer.
 * @retval None
 */
static void host_process_rx_byte(uint8_t rx_byte){
    static Rx_ParserState_t state = RX_STATE_FIND_START;
    static uint16_t         data_idx = 0U;
    static uint8_t          packet_parsing_buf[PSPM_PACKET_SIZE];

    switch(state){
        case RX_STATE_FIND_START:
            if(rx_byte == PSPM_START_BYTE_RX){ // Match Header 0x55
                packet_parsing_buf[0] = rx_byte;
                data_idx = 0U;
                state = RX_STATE_COLLECT_DATA;
            }
            break;

        case RX_STATE_COLLECT_DATA:
            packet_parsing_buf[1 + data_idx] = rx_byte;
            data_idx++;
            if(data_idx >= PSPM_DATA_SIZE){
                state = RX_STATE_GET_CRC_L;
            }
            break;

        case RX_STATE_GET_CRC_L:
            packet_parsing_buf[29] = rx_byte;
            state = RX_STATE_GET_CRC_H;
            break;

        case RX_STATE_GET_CRC_H:
            packet_parsing_buf[30] = rx_byte;
            state = RX_STATE_VERIFY_XOR;
            break;

        case RX_STATE_VERIFY_XOR:
            packet_parsing_buf[31] = rx_byte;

            // 1. Verify CRC16 calculated over Bytes 0 to 28 (Total 29 Bytes)
            uint16_t expected_crc = compute_crc16(packet_parsing_buf, 29U);
            uint16_t received_crc = ((uint16_t)packet_parsing_buf[30] << 8) | packet_parsing_buf[29];

            if(expected_crc == received_crc){
                // 2. Verify XOR_ALL checksum calculated over Bytes 0 to 30 (Total 31 Bytes)
                uint8_t expected_xor = compute_xor(packet_parsing_buf, 31U);
                uint8_t received_xor = packet_parsing_buf[31];

                if(expected_xor == received_xor){
                    last_bc_packet_tick = osKernelGetTickCount();
                    is_bc_link_error    = false;
                    host_stats.rx_count++;

                    // 3. Extract 10 DAC setpoints from validated packet
                    Bc_To_Pspm_Data_t bc_data;
                    memcpy(&bc_data, &packet_parsing_buf[1], sizeof(Bc_To_Pspm_Data_t));

                    FpgaDmaDacPacket_t dac_packet;
                    dac_packet.dac_val[0] = bc_data.asel_a;      // DAC 1: DUVN_AC
                    dac_packet.dac_val[1] = bc_data.asel_d;      // DAC 2: DUVN_DC
                    dac_packet.dac_val[2] = bc_data.asaz_a;      // DAC 3: DUGN_AC
                    dac_packet.dac_val[3] = bc_data.asaz_d;      // DAC 4: DUGN_DC
                    dac_packet.dac_val[4] = bc_data.scale_asel;  // DAC 5: Scale DUVN
                    dac_packet.dac_val[5] = bc_data.scale_gps;   // DAC 6: Scale DPT
                    dac_packet.dac_val[6] = bc_data.adj_0_alpha; // DAC 7: Reg 0 + Alpha
                    dac_packet.dac_val[7] = bc_data.scale_asaz;  // DAC 8: Scale DUGN
                    dac_packet.dac_val[8] = bc_data.scale_hps;   // DAC 9: Scale DPL
                    dac_packet.dac_val[9] = bc_data.adj_0_beta;  // DAC 10: Reg 0 + Beta

                    // 4. Commit DAC setpoints to FPGA via SPI DMA unless the
                    //    debug waveform generator currently owns the DAC outputs
                    if (!dbg_gen_dac_busy()) {
                        IC_WriteDacSetpoints(&dac_packet, 10);
                    }
                } else {
                    host_stats.rx_xor_err++;
                }
            } else {
                host_stats.rx_crc_err++;
            }
            state = RX_STATE_FIND_START;
            break;

        default:
            state = RX_STATE_FIND_START;
            break;
    }
}

/* ========================================================================= */
/*  LOW-LEVEL DMA UART DRIVER                                                */
/* ========================================================================= */

/**
 * @brief  Initializes UART synchronization mutex and DMA transmission semaphore.
 * @param  huart: Pointer to HAL UART handle (USART2).
 * @retval true on successful allocation, false otherwise.
 */
bool host_uart_init(UART_HandleTypeDef *huart){
    if(!huart) return false;
    host_ctx.huart = huart;

    const osMutexAttr_t mutex_attr = { .name = "HostTxMutex" };
    host_ctx.tx_mutex = osMutexNew(&mutex_attr);

    const osSemaphoreAttr_t sem_attr = { .name = "HostTxSem" };
    host_ctx.tx_sem   = osSemaphoreNew(1, 1, &sem_attr);

    host_ctx.rx_sem   = osSemaphoreNew(1, 0, NULL);

    return (host_ctx.tx_mutex != NULL && host_ctx.tx_sem != NULL
            && host_ctx.rx_sem != NULL);
}

/**
 * @brief  Arms USART2 DMA in ReceiveToIdle mode.
 * @note   Disables half-transfer interrupt to eliminate redundant ISR overhead.
 * @retval None
 */
void host_uart_start_receive(void){
    HAL_UARTEx_ReceiveToIdle_DMA(host_ctx.huart, host_rx_buf, HOST_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(host_ctx.huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief  Sends raw byte buffer via USART2 DMA with thread-safety and OS task yield.
 * @param  data: Pointer to packet buffer.
 * @param  len: Byte length (typically 32 bytes).
 * @retval true if transmission started successfully, false on timeout/error.
 */
static bool host_uart_send_raw(const uint8_t *data, uint16_t len){
    if(len == 0 || len > HOST_TX_BUF_SIZE){
        return false;
    }

    if(osMutexAcquire(host_ctx.tx_mutex, osWaitForever) != osOK){
        return false;
    }

    // Wait for previous DMA TX to complete (Blocks thread, 0% CPU)
    if(osSemaphoreAcquire(host_ctx.tx_sem, 100) != osOK){
        osMutexRelease(host_ctx.tx_mutex);
        return false;
    }

    memcpy(host_tx_buf, data, len);

    if(HAL_UART_Transmit_DMA(host_ctx.huart, host_tx_buf, len) != HAL_OK){
        osSemaphoreRelease(host_ctx.tx_sem);
        osMutexRelease(host_ctx.tx_mutex);
        return false;
    }

    osMutexRelease(host_ctx.tx_mutex);
    return true;
}

/**
 * @brief  DMA TX Complete ISR Callback. Releases semaphore to unblock transmitter.
 * @param  huart: Pointer to HAL UART handle.
 * @retval None
 */
void host_uart_tx_complete_handler(UART_HandleTypeDef *huart){
    if(huart == host_ctx.huart){
        osSemaphoreRelease(host_ctx.tx_sem);
    }
}

/**
 * @brief  UART ISR body: queue raw bytes into the SPSC ring, re-arm DMA reception.
 * @note   No parsing, no FPGA access, no blocking OS calls here (FreeRTOS ISR rules).
 *         Consumer task drains the ring via host_process_rx_pending().
 * @param  huart: Pointer to HAL UART handle.
 * @param  Size: Number of newly received bytes.
 * @retval None
 */
void host_uart_rx_event_handler(UART_HandleTypeDef *huart, uint16_t Size){
    if(huart == host_ctx.huart){
        for(uint16_t i = 0; i < Size; i++){
            if((host_rx_wr - host_rx_rd) < HOST_RX_RING_SIZE){
                host_rx_ring[host_rx_wr & (HOST_RX_RING_SIZE - 1U)] = host_rx_buf[i];
                host_rx_wr++;
            } else {
                host_stats.rx_drop++;   /* consumer stalled >80 ms: byte lost */
            }
        }
        host_uart_start_receive();
        /* Wake the consumer task immediately (ISR-safe in CMSIS-RTOS2) */
        osSemaphoreRelease(host_ctx.rx_sem);
    }
}

/**
 * @brief  Task-context drain: parses all bytes queued by the UART ISR.
 * @note   Must be called from task context only (400 Hz main loop): the FSM
 *         commits validated DAC packets through IC_WriteDacSetpoints (SPI/DMA).
 * @retval None
 */
void host_process_rx_pending(void){
    while(host_rx_rd != host_rx_wr){
        uint8_t byte = host_rx_ring[host_rx_rd & (HOST_RX_RING_SIZE - 1U)];
        host_rx_rd++;
        host_process_rx_byte(byte);
    }
}

/**
 * @brief  Blocks the caller until the UART ISR queues bytes or timeout expires.
 * @param  timeout_ticks: RTOS tick timeout.
 * @retval RTOS semaphore status.
 */
osStatus_t host_rx_wait(uint32_t timeout_ticks){
    if(host_ctx.rx_sem == NULL){
        return osErrorResource;
    }
    return osSemaphoreAcquire(host_ctx.rx_sem, timeout_ticks);
}

/**
 * @brief  RX guard: re-arms one-shot DMA reception if it was lost
 *         (e.g. after a UART error path). Call periodically from a task.
 * @retval None
 */
void host_rx_guard(void){
    if(host_ctx.huart == NULL){
        return;
    }
    if(host_ctx.huart->RxState == HAL_UART_STATE_READY){
        host_uart_start_receive(); /* Idle DMA reception was never armed */
    }
}

/**
 * @brief  Hardware error recovery handler for UART / DMA.
 * @note   Clears Overrun (ORE) flags and restarts reception.
 * @param  huart: Pointer to HAL UART handle.
 * @retval None
 */
void host_uart_error_handler(UART_HandleTypeDef *huart){
    if(huart == host_ctx.huart){
        HAL_UART_DMAStop(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        osSemaphoreRelease(host_ctx.tx_sem);
        host_uart_start_receive();
    }
}

/* ========================================================================= */
/*  HIGH-LEVEL PSPM -> BC PROTOCOL ENCODING                                  */
/* ========================================================================= */

/**
 * @brief  Assembles complete 32-byte telemetry packet and transmits over USART2 DMA.
 * @note   Packs:
 *          - Scaled Peak Tracking Errors (Vertical/Azimuth) in arcminutes.
 *          - Angular Velocities W_EL / W_AZ (+-5 deg/s).
 *          - Gun Position, Tank Speed, Wind Speed, Turret Cosine.
 *          - Roll and Pitch angles in BAM16 format.
 *          - Atmospheric Pressure in mmHg from BMP280.
 *          - System Health Status Byte.
 *          - CRC-16-CCITT and XOR_ALL Checksums.
 * @retval None
 */
void host_send_pspm_packet(void){
    Pspm_Packet_t      packet;
    Pspm_To_Bc_Data_t  data;

    memset(&packet, 0, sizeof(Pspm_Packet_t));
    memset(&data, 0, sizeof(Pspm_To_Bc_Data_t));

    const rdc_telemetry_t      *rdc = RDC_GetTelemetry();
    const fpga_adc_telemetry_t *adc = FPGA_ADC_GetTelemetry();

    // 1. Elevation & Azimuth Peak Tracking Errors (Direct int16_t from FPGA)
    data.err_el    = adc->raw_16b.err_el;
    data.err_el_10 = adc->raw_16b.err_el_10;
    data.err_az    = adc->raw_16b.err_az;
    data.err_az_10 = adc->raw_16b.err_az_10;

    // 2. Angular Velocities W_EL & W_AZ (+-5 deg/s -> +-32768)
    // 1 LSB = 5.0 / 32768 = 0.00015258789 deg/s
    float raw_w_el = (rdc->pitch_speed_dps / 5.0f) * 32768.0f;
    float raw_w_az = (rdc->roll_speed_dps / 5.0f) * 32768.0f;
    data.w_el = (int16_t)CLAMP(raw_w_el, -32768.0f, 32767.0f);
    data.w_az = (int16_t)CLAMP(raw_w_az, -32768.0f, 32767.0f);

    // 3. Gun Position, Tank Speed, Wind Speed, Turret Cosine
    data.gun_pos = adc->raw_16b.gun_pos;
    data.v_tank  = adc->raw_16b.v_tank;
    data.v_wind  = adc->raw_16b.v_wind;
    data.cos_fi  = adc->raw_16b.cos_fi;

    // 4. Roll & Pitch Angles in BAM16 (-180..+180 deg -> -32768..+32767)
    data.roll  = rdc->roll_angle_bf;
    data.pitch = rdc->pitch_angle_bf;

    // 5. Atmospheric Pressure P_A (0..1500 mmHg -> 0..65535)
    // 1 LSB = 1500 / 65536 = 0.02288818 mmHg (1 Pa = 0.00750062 mmHg)
    // Validity-gated: no fresh sample -> p_a = 0 (impossible value) AND the
    // PRESSURE_ERROR status bit, so BC sees an explicit fault, never stale data.
    BMP280_Data_t bmp;
    bool bmp_valid = BMP280_GetTelemetry(&bmp);
    if (bmp_valid){
        float p_mmhg = bmp.pressure_pa * 0.00750062f;
        float raw_pa = (p_mmhg / 1500.0f) * 65536.0f;
        data.p_a = (uint16_t)CLAMP(raw_pa, 0.0f, 65535.0f);
    } else {
        data.p_a = 0U;
    }

    // 6. System Status Byte (Byte 26) & Reserved Padding (Byte 27)
    data.status   = IC_GetSystemStatusByte();
    if (!bmp_valid) {
        data.status |= PSPM_STATUS_PRESSURE_ERROR;
    }
    data.reserved = 0x00U;

    // 7. Build 32-Byte Packet Frame Layout
    packet.start = PSPM_START_BYTE_TX; // 0xAA Header
    memcpy(packet.data, &data, PSPM_DATA_SIZE);

    // 8. Calculate CRC-16-CCITT over Bytes 0..28 (29 Bytes)
    uint16_t crc_val = compute_crc16((uint8_t*)&packet, 29U);
    packet.crc_l     = (uint8_t)(crc_val & 0x00FFU);
    packet.crc_h     = (uint8_t)((crc_val >> 8) & 0x00FFU);

    // 9. Calculate XOR_ALL over Bytes 0..30 (31 Bytes)
    packet.xor_all = compute_xor((uint8_t*)&packet, 31U);

    // 10. Start thread-safe non-blocking DMA transmission over USART2
    if(host_uart_send_raw((const uint8_t*)&packet, PSPM_PACKET_SIZE) == true){
        host_stats.tx_count++;
    }
}

/**
 * @brief  Master Host Process wrapper called by the 400 Hz control sequencer.
 * @retval None
 */
void HOST_Process(void){
    host_send_pspm_packet();
}

/* ========================================================================= */
/*  HAL RX EVENT CALLBACK ROUTING                                            */
/* ========================================================================= */

/**
 * @brief  HAL Extended RX Event Callback routing for USART2.
 * @param  huart: Pointer to HAL UART handle.
 * @param  Size: Transfer size.
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if(huart->Instance == USART2){
        host_uart_rx_event_handler(huart, Size);
    }
}
