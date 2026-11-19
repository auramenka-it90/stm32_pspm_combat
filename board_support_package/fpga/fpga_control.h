/**
 ******************************************************************************
 * @file    fpga_control.h
 * @brief   Header for High-Level FPGA Supervisory & Debug Control API for PSPM.
 *          Target: Xilinx Spartan-6 + STM32F4 (SPI Mode 00).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef FPGA_CONTROL_H
#define FPGA_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "stm32_2_fpga_spi_bridge.h"
#include "fpga_reg_map.h"

/* RS-485 Channel Selection (Bit 3 of REG_MISC) */
typedef enum {
    FPGA_UART_PORT_DD4 = 0, /* Channel 1: Transceiver DD4 (tx2, rx2, de2) */
    FPGA_UART_PORT_DD5 = 1  /* Channel 2: Transceiver DD5 (tx3, rx3, de3) */
} FPGA_Uart_Port_t;

/* Resolver Reference Phase Selection (Bit 4 of REG_MISC) */
typedef enum {
    FPGA_RES_PHASE_UON12 = 0, /* Phase 1-2 Comparator (0 deg reference) */
    FPGA_RES_PHASE_UON23 = 1  /* Phase 2-3 Comparator (120 deg shifted) */
} FPGA_Resolver_Phase_t;

/* ========================================================================= */
/*  DEBUG & SYSTEM SUPERVISORY APIS (Device ID = 1)                          */
/* ========================================================================= */

/**
 * @brief  Verifies SPI connection by checking hardcoded 0xDEAD validation constant.
 */
FPGA_Status_t FPGA_Debug_Verify(FPGA_HandleTypeDef *hbridge, uint32_t timeout_ms);

/**
 * @brief  Tests SPI bus integrity by writing and reading back an echo pattern (0x55AA).
 */
FPGA_Status_t FPGA_Debug_Test_Echo(FPGA_HandleTypeDef *hbridge, uint16_t test_val, uint32_t timeout_ms);

/**
 * @brief  Updates physical status LEDs (VD35..37) via thread-safe Read-Modify-Write.
 */
FPGA_Status_t FPGA_Debug_Set_LEDs(FPGA_HandleTypeDef *hbridge, uint8_t led_mask, uint32_t timeout_ms);

/**
 * @brief  Direct boolean control of Red (VD37), Yellow (VD36), and Green (VD35) LEDs.
 */
FPGA_Status_t FPGA_Debug_Write_LEDs(FPGA_HandleTypeDef *hbridge, bool r, bool y, bool g, uint32_t timeout_ms);

/**
 * @brief  Switches active RS-485 transceiver: 0 = DD4, 1 = DD5.
 */
FPGA_Status_t FPGA_Debug_Set_UART_Mux(FPGA_HandleTypeDef *hbridge, FPGA_Uart_Port_t port, uint32_t timeout_ms);

/**
 * @brief  Selects Resolver reference phase hardware multiplexer (0 = UON12, 1 = UON23).
 */
FPGA_Status_t FPGA_Debug_Set_Resolver_Phase(FPGA_HandleTypeDef *hbridge, FPGA_Resolver_Phase_t phase, uint32_t timeout_ms);

/**
 * @brief  Enables/Disables internal 400 Hz 3-Phase BIST generator (0 = Ext Pins, 1 = BIST).
 */
FPGA_Status_t FPGA_Debug_Set_BIST_Enable(FPGA_HandleTypeDef *hbridge, bool enable, uint32_t timeout_ms);

/**
 * @brief  Configures dynamic signal routing for diagnostic testpoints TP5, TP6, TP7.
 * @param  tp5_sig: Signal index for TP5 (0..29, see FPGA_TP_SIG_* in fpga_reg_map.h)
 * @param  tp6_sig: Signal index for TP6 (0..29)
 * @param  tp7_sig: Signal index for TP7 (0..29)
 */
FPGA_Status_t FPGA_Debug_Set_TP_Mux(FPGA_HandleTypeDef *hbridge, uint8_t tp5_sig, uint8_t tp6_sig, uint8_t tp7_sig, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* FPGA_CONTROL_H */
