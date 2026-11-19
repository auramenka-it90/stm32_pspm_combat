/**
 ******************************************************************************
 * @file    stm32_2_fpga_spi_bridge.h
 * @brief   High-Speed SPI Driver for STM32 to Spartan-6 FPGA Communication.
 *          Supports Polling, Atomic Bit Manipulations, and DMA Burst modes.
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2 + HAL).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef STM32_2_FPGA_SPI_BRIDGE_H
#define STM32_2_FPGA_SPI_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "pin_mgmt.h"
#include "fpga_reg_map.h"
#include <stdbool.h>

/* ========================================================================= */
/*  TYPE DEFINITIONS & RETURN CODES                                          */
/* ========================================================================= */

typedef enum {
    FPGA_OK      = 0,
    FPGA_ERROR   = 1,
    FPGA_TIMEOUT = 2,
    FPGA_BUSY    = 3
} FPGA_Status_t;

/* Main FPGA Bridge Handle Structure */
typedef struct {
    SPI_HandleTypeDef      *hspi;           /* Pointer to physical SPI peripheral */

    /* Dedicated Dual-Plane Chip Select Pins */
    const Pin_Descriptor_t *nss_poll_pin;   /* Pin for Polling mode (PB0 / NSS_P) */
    const Pin_Descriptor_t *nss_dma_pin;    /* Pin for DMA Burst mode (PB2 / NSS_D) */

    /* RTOS Synchronization Primitives */
    osMutexId_t             mutex_id;       /* Protects SPI bus from concurrent tasks */
    osSemaphoreId_t         dma_sem_id;     /* Binary semaphore for DMA completion */

    /* Active Transfer State Tracking */
    const Pin_Descriptor_t *active_nss_pin; /* Pointer to currently active CS pin */

    /* 4-byte aligned buffers for SPI Polling (Control Plane) */
    uint16_t                tx_poll_buf[2] __attribute__((aligned(4)));
    uint16_t                rx_poll_buf[2] __attribute__((aligned(4)));

    /* 4-byte aligned raw DMA frame buffers (Data Plane) */
    FpgaDmaTxFrame_t        dma_tx_frame   __attribute__((aligned(4))); /* 11 words = 22 bytes */
    FpgaDmaRxFrame_t        dma_rx_frame   __attribute__((aligned(4))); /* 30 words = 60 bytes */
} FPGA_HandleTypeDef;

/* Global handle reference for ISR callbacks */
extern FPGA_HandleTypeDef *g_fpga_bridge;

/* ========================================================================= */
/*  CONVENIENCE BIT MANIPULATION MACROS                                      */
/* ========================================================================= */

#define FPGA_SET_BIT(hbridge, addr, mask)       FPGA_Set_Bit_Poll((hbridge), (addr), (mask), 100)
#define FPGA_CLR_BIT(hbridge, addr, mask)       FPGA_Clear_Bit_Poll((hbridge), (addr), (mask), 100)
#define FPGA_TOGGLE_BIT(hbridge, addr, mask)    FPGA_Toggle_Bit_Poll((hbridge), (addr), (mask), 100)

/* ========================================================================= */
/*  PUBLIC API PROTOTYPES                                                    */
/* ========================================================================= */

/* Initialization */
FPGA_Status_t   FPGA_Bridge_Init(FPGA_HandleTypeDef *hbridge,
                                 SPI_HandleTypeDef  *hspi,
                                 const Pin_Descriptor_t *nss_poll,
                                 const Pin_Descriptor_t *nss_dma);

/* Synchronous 16-Bit Register Access (Control Plane - NSS_P) */
FPGA_Status_t   FPGA_Write_Poll(FPGA_HandleTypeDef *hbridge,
                                uint16_t addr,
                                uint16_t data,
                                uint32_t timeout_ms);

FPGA_Status_t   FPGA_Read_Poll(FPGA_HandleTypeDef *hbridge,
                               uint16_t addr,
                               uint16_t *data,
                               uint32_t timeout_ms);

/* Atomic Read-Modify-Write Bit Operations */
FPGA_Status_t   FPGA_Set_Bit_Poll(FPGA_HandleTypeDef *hbridge,
                                  uint16_t addr,
                                  uint16_t bit_mask,
                                  uint32_t timeout_ms);

FPGA_Status_t   FPGA_Clear_Bit_Poll(FPGA_HandleTypeDef *hbridge,
                                    uint16_t addr,
                                    uint16_t bit_mask,
                                    uint32_t timeout_ms);

FPGA_Status_t   FPGA_Toggle_Bit_Poll(FPGA_HandleTypeDef *hbridge,
                                     uint16_t addr,
                                     uint16_t bit_mask,
                                     uint32_t timeout_ms);

FPGA_Status_t   FPGA_Modify_Bits_Poll(FPGA_HandleTypeDef *hbridge,
                                      uint16_t addr,
                                      uint16_t clear_mask,
                                      uint16_t set_mask,
                                      uint32_t timeout_ms);

FPGA_Status_t   FPGA_Is_Bit_Set_Poll(FPGA_HandleTypeDef *hbridge,
                                     uint16_t addr,
                                     uint16_t bit_mask,
                                     bool *is_set,
                                     uint32_t timeout_ms);

/* High-Speed DMA Streaming Transfers (Data Plane - NSS_D) */
FPGA_Status_t   FPGA_Read_Telemetry_DMA(FPGA_HandleTypeDef *hbridge,
                                       FpgaDmaTelemetryPacket_t *telemetry_out,
                                       uint32_t timeout_ms);

FPGA_Status_t   FPGA_Write_DAC_DMA(FPGA_HandleTypeDef *hbridge,
                                   const FpgaDmaDacPacket_t *dac_in,
                                   uint32_t timeout_ms);

FPGA_Status_t   FPGA_Exchange_FullDuplex_DMA(FPGA_HandleTypeDef *hbridge,
                                            const FpgaDmaDacPacket_t *dac_in,
                                            FpgaDmaTelemetryPacket_t *telemetry_out,
                                            uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* STM32_2_FPGA_SPI_BRIDGE_H */
