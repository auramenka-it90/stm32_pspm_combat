/**
 ******************************************************************************
 * @file    ic.h
 * @brief   Declarations for Interrupt Controller & Raw DMA Telemetry Dispatcher.
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef IC_H
#define IC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pin_mgmt.h"
#include "stm32_2_fpga_spi_bridge.h"
#include "fpga_reg_map.h"
#include <stdbool.h>
#include <stdint.h>

/* Thread event flag set by the EXTI ISR for ANY FPGA interrupt wire falling
   (data-ready OR frequency-error are OR'd onto PB10 inside the FPGA) */
#define IC_EVENT_FLAG_FPGA_IRQ         0x00000001U
#define IC_400HZ_TIMEOUT_MS          10U

#define IC_STATUS_GLOBAL_ERROR       0x01U
#define IC_STATUS_ADC_ERROR          0x02U
#define IC_STATUS_FPGA_ERROR         0x04U
#define IC_STATUS_OK                 0x00U

/* FPGA Sticky Status Event Indices (Burst Word 33 rising edges) */
typedef enum {
    IC_ST_EVT_ADC_ERR = 0,       /* Watchdog timeout DA21/DA22/DA45   */
    IC_ST_EVT_RES_ERR,           /* Resolver ADC watchdog timeout     */
    IC_ST_EVT_RES_OVR,           /* Resolver overrun                  */
    IC_ST_EVT_ERR_12,            /* Comparator 1-2 error              */
    IC_ST_EVT_ERR_23,            /* Comparator 2-3 error              */
    IC_ST_EVT_COUNT
} IC_StEvt_e;

/* Initialization & Task Binding */
bool          IC_Init(FPGA_HandleTypeDef *hbridge, osThreadId_t worker_task, uint32_t event_flag);
void          IC_AttachTask(osThreadId_t worker_task, uint32_t event_flag);

/* Public diagnostic storage (updated atomically under mutex inside IC_Process,
   bound directly to DSPA terminal signals) */
extern uint16_t g_fpga_status_word;
extern uint32_t g_fpga_st_evt[IC_ST_EVT_COUNT];

/* Real-Time 400 Hz Execution Engine */
bool          IC_Process(uint32_t timeout_ms);

/* Thread-Safe Raw Data Access */
bool          IC_GetRawTelemetry(FpgaDmaTelemetryPacket_t *dest);
FPGA_Status_t IC_WriteDacSetpoints(const FpgaDmaDacPacket_t *dac_data, uint32_t timeout_ms);

/* Diagnostics */
bool          IC_Is400HzValid(void);
uint8_t       IC_GetSystemStatusByte(void);
uint32_t      IC_GetFrameCount(void);
uint16_t      IC_GetStatusWord(void);
uint32_t      IC_GetStatusEvtCount(uint8_t idx);
void          IC_ResetStatusEvtCounts(void);

/* FPGA Device ID 2 Low-Level Control */
FPGA_Status_t IC_FPGA_Configure(uint16_t mask, uint16_t edge_sel, bool global_en);
FPGA_Status_t IC_FPGA_GetPending(uint16_t *pending);
FPGA_Status_t IC_FPGA_ClearPending(uint16_t clear_mask);

#ifdef __cplusplus
}
#endif

#endif /* IC_H */
