/**
 ******************************************************************************
 * @file    host.h
 * @brief   Declarations for PSPM <-> BC Serial Protocol Driver (USART2, RS-422).
 *          Baudrate: 921600, 8 Bits, Even Parity, 1 Stop Bit.
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef HOST_H
#define HOST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "board_support_package.h"
#include "fpga_reg_map.h"
#include "rdc.h"
#include "fpga_adc.h"
#include "bmp280.h"
#include "ic.h"
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"

/* ========================================================================= */
/*  PROTOCOL SPECIFICATION CONSTANTS                                         */
/* ========================================================================= */

#define PSPM_UART_BAUDRATE           921600U
#define PSPM_PACKET_SIZE             32U
#define PSPM_DATA_SIZE               28U
#define PSPM_CRC_SIZE                2U

#define PSPM_START_BYTE_TX           0xAAU  /* Outgoing packet header (PSPM -> BC) */
#define PSPM_START_BYTE_RX           0x55U  /* Incoming packet header (BC -> PSPM) */

#define PSPM_CRC_POLYNOMIAL          0x1021U
#define PSPM_CRC_INIT                0xFFFFU

#define HOST_RX_BUF_SIZE             128U   /* Size of double packet receive DMA buffer */
#define HOST_TX_BUF_SIZE             128U   /* Size of thread-safe transmit DMA buffer */

/* D[26] status byte: Bit 3 = pressure sensor fault. Bits 0..2 are produced by
   IC_GetSystemStatusByte() (FPGA/IC health); this bit is set by the host layer
   when BMP280 has no fresh valid sample (0 in p_a is the paired sentinel). */
#define PSPM_STATUS_PRESSURE_ERROR   0x08U

/* ========================================================================= */
/*  PACKET STRUCTURE DEFINITIONS (PACKED, LITTLE-ENDIAN)                     */
/* ========================================================================= */

/**
 * @brief  PSPM -> BC Data Field Structure (28 bytes: D[0]..D[27])
 */
typedef struct __attribute__((packed)) {
    int16_t  err_el;          /* D[0..1]   : Vertical Error (+-400 arcmin, 1 LSB = 0.012207') */
    int16_t  err_el_10;       /* D[2..3]   : Vertical Error x10 (+-40 arcmin, 1 LSB = 0.0012207') */
    int16_t  err_az;          /* D[4..5]   : Azimuth Error (+-400 arcmin, 1 LSB = 0.012207') */
    int16_t  err_az_10;       /* D[6..7]   : Azimuth Error x10 (+-40 arcmin, 1 LSB = 0.0012207') */
    int16_t  w_el;            /* D[8..9]   : Vertical Angular Velocity (+-5 deg/s) */
    int16_t  w_az;            /* D[10..11] : Azimuth Angular Velocity (+-5 deg/s) */
    int16_t  gun_pos;         /* D[12..13] : Gun Position (-15..+15 deg) */
    int16_t  roll;            /* D[14..15] : Roll Angle BAM16 (-180..+180 deg) */
    int16_t  pitch;           /* D[16..17] : Pitch Angle BAM16 (-180..+180 deg) */
    int16_t  v_tank;          /* D[18..19] : Tank Speed (-20..+20 m/s) */
    int16_t  v_wind;          /* D[20..21] : Crosswind Speed (-25..+25 m/s) */
    int16_t  cos_fi;          /* D[22..23] : Turret Rotation Cosine (-1.0..+1.0) */
    uint16_t p_a;             /* D[24..25] : Atmospheric Pressure (0..1500 mmHg) */
    uint8_t  status;          /* D[26]     : Bit0 Global, Bit1 ADC, Bit2 FPGA, Bit3 Pressure Err */
    uint8_t  reserved;        /* D[27]     : Reserved padding (always 0x00) */
} Pspm_To_Bc_Data_t;

/**
 * @brief  BC -> PSPM Data Field Structure (28 bytes: D[0]..D[27] -> 10 DACs)
 */
typedef struct __attribute__((packed)) {
    int16_t  asel_a;          /* D[0..1]   : DAC 1: DUVN_AC (Elevation Angle Sensor AC) */
    int16_t  asel_d;          /* D[2..3]   : DAC 2: DUVN_DC (Elevation Angle Sensor DC) */
    int16_t  asaz_a;          /* D[4..5]   : DAC 3: DUGN_AC (Azimuth Angle Sensor AC) */
    int16_t  asaz_d;          /* D[6..7]   : DAC 4: DUGN_DC (Azimuth Angle Sensor DC) */
    int16_t  scale_asel;      /* D[8..9]   : DAC 5: Scale DUVN */
    int16_t  scale_gps;       /* D[10..11] : DAC 6: Scale DPT (Gun Position Scale) */
    int16_t  adj_0_alpha;     /* D[12..13] : DAC 7: Adj 0 + Elevation Correction */
    int16_t  scale_asaz;      /* D[14..15] : DAC 8: Scale DUGN */
    int16_t  scale_hps;       /* D[16..17] : DAC 9: Scale DPL (Hatch Position Scale) */
    int16_t  adj_0_beta;      /* D[18..19] : DAC 10: Adj 0 + Azimuth Correction */
    uint8_t  reserved[8];     /* D[20..27] : Reserved padding (always 0x00) */
} Bc_To_Pspm_Data_t;

/**
 * @brief  Complete 32-byte Physical Frame Structure
 */
typedef struct __attribute__((packed)) {
    uint8_t  start;           /* Byte 0    : Start Byte (0xAA for TX, 0x55 for RX) */
    uint8_t  data[PSPM_DATA_SIZE]; /* Bytes 1..28: 28-byte Data Payload */
    uint8_t  crc_l;           /* Byte 29   : CRC16 Little-Endian LSB */
    uint8_t  crc_h;           /* Byte 30   : CRC16 Little-Endian MSB */
    uint8_t  xor_all;         /* Byte 31   : Cumulative XOR Checksum */
} Pspm_Packet_t;

/* ========================================================================= */
/*  STATISTICS & DIAGNOSTICS                                                 */
/* ========================================================================= */

typedef struct {
    uint32_t tx_count;        /* Total transmitted packets (PSPM -> BC) */
    uint32_t rx_count;        /* Total successfully validated packets (BC -> PSPM) */
    uint32_t rx_crc_err;      /* Discarded packets due to CRC16 failure */
    uint32_t rx_xor_err;      /* Discarded packets due to XOR failure */
    uint32_t rx_drop;         /* Bytes lost: SPSC ring full (consumer stalled >80 ms) */
} Host_Stats_t;

extern Host_Stats_t      host_stats;
extern volatile bool     is_bc_link_error;
extern volatile uint32_t last_bc_packet_tick;

/* ========================================================================= */
/*  PUBLIC API PROTOTYPES                                                    */
/* ========================================================================= */

bool                host_uart_init(UART_HandleTypeDef *huart);
void                host_uart_start_receive(void);
void                HOST_Process(void);
void                host_send_pspm_packet(void);

/* Interrupt Handlers */
void                host_uart_rx_event_handler(UART_HandleTypeDef *huart, uint16_t Size);
void                host_uart_tx_complete_handler(UART_HandleTypeDef *huart);
void                host_uart_error_handler(UART_HandleTypeDef *huart);

/* RX consumer API (task context): drain SPSC ring, wait for bytes, guard re-arm */
void                host_process_rx_pending(void);
osStatus_t          host_rx_wait(uint32_t timeout_ticks);
void                host_rx_guard(void);

#ifdef __cplusplus
}
#endif

#endif /* HOST_H */
