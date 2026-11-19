/**
 ******************************************************************************
 * @file    terminal.h
 * @brief   DSPAssist Terminal Communication and Diagnostic Interface for PSPM.
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "board_support_package.h"

#define _TERMINAL_PERMISSION_WRAPPER_

#ifdef _TERMINAL_PERMISSION_WRAPPER_
    /* Wrapper Address: PUBV = 10, BC = 11, MSB = 12, PSPM = 13 (0x0D) */
    #define _TERMINAL_WRAPPER_ADDRESS_      0x0DU
    #define _TERMINAL_WRAPPER_ALT_ADDRESS_  _TERMINAL_WRAPPER_ADDRESS_
#endif

/* Buffer sizing & timing constants */
#define _TERMINAL_RX_BUFF_SIZE_             (1024U * 2U)
#define _TERMINAL_TX_BUFF_SIZE_             (1024U * 8U)
#define _TERMINAL_TASK_TICK_                1U       /* Period in ms */
#define _TERMINAL_RECEIVE_TIMEOUT_US_       20000U   /* Timeout in us */
#define _TERMINAL_BOOTLOADER_TIMEOUT_       15000U

/* PSPM has 6 active SPI modules */
#define _TERMINAL_FPGA_MAX_NUM_BLOCK_       6U

/* Memory Page Access Limits for DSPA */
#define DSPA_CFG_WRITE_PAGE_SIZE            1024U
#define DSPA_CFG_READ_PAGE_SIZE             1024U

/* ========================================================================= */
/*  PUBLIC API PROTOTYPES                                                    */
/* ========================================================================= */

bool terminal_init(void);
void terminal_task(void);
void terminal_telemetry_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* TERMINAL_H_ */
