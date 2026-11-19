/**
 ******************************************************************************
 * @file    terminal_signals.h
 * @brief   Header for DSPAssist Terminal Telemetry Signals for PSPM.
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef TERMINAL_SIGNALS_H_
#define TERMINAL_SIGNALS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "board_support_package.h"
#include "host.h"
#include "configuration.h"
#include "rdc.h"
#include "fpga_adc.h"
#include "bmp280.h"
#include "stm32_2_fpga_spi_bridge.h"

#define DSPA_SIGNALS_NAME   dspa

/* Task execution counter */
extern uint32_t pspm_task_counter;

/* Global FPGA Bridge Handle from board_support_package.c */
extern FPGA_HandleTypeDef hfpga_bridge;

/* Public APIs */
int  init_terminal_signals(void);
void terminal_update_dynamic_strings(void); /* <-- Объявление функции */

#ifdef __cplusplus
}
#endif

#endif /* TERMINAL_SIGNALS_H_ */
