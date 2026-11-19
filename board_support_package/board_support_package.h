/**
 ******************************************************************************
 * @file    board_support_package.h
 * @brief   Board Support Package (BSP) global configuration registry.
 *          Target MCU: STM32F411RET6 (100 MHz, FreeRTOS CMSIS-OS v2).
 *          Board: PSPM (DIAS.901.02.00.300).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef BOARD_SUPPORT_PACKAGE_H
#define BOARD_SUPPORT_PACKAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "main.h"
#include "adc.h"
#include "cmsis_os2.h"
#include "usefull_define.h"


/* 0: Standalone debug without bootloader (0x08000000, linker script STM32F411RETX_FLASH.ld)
 * 1: Run under Bootloader control (0x08020000, linker script STM32F411RETX_FLASH_BL.ld,
 *    set Script file in Properties -> MCU GCC Linker -> General to STM32F411RETX_FLASH_BL.ld) */
#define APPLICATION_USE_BOOTLOADER      0


/* ========================================================================= */
/*  STM32 HARDWARE PERIPHERAL HANDLES                                        */
/* ========================================================================= */

/* SPI handles */
extern SPI_HandleTypeDef  hspi1;
extern SPI_HandleTypeDef  hspi3;

/* UART1 handle and DMA channels (KPA / RS-485) */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart1_tx;

/* UART6 handle and DMA channels (ST-Link Virtual COM Port) */
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef  hdma_usart6_rx;
extern DMA_HandleTypeDef  hdma_usart6_tx;

/* UART2 handle and DMA channels (Ballistic Computer link via FPGA) */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern DMA_HandleTypeDef  hdma_usart2_tx;

/* ADC handle for CPU temperature and analog measurements */
extern ADC_HandleTypeDef  hadc1;

/* Hardware CRC peripheral handle */
#ifdef HAL_CRC_MODULE_ENABLED
extern CRC_HandleTypeDef  hcrc;
#endif



/* ========================================================================= */
/*  GLOBAL HARDWARE TELEMETRY & DIAGNOSTICS                                  */
/* ========================================================================= */

extern uint32_t test_hardware_result; /* Global POST diagnostic status register */
extern float    adc_voltage;          /* Temperature sensor voltage, V */
extern float    cpu_temperature;      /* Die temperature, deg C */
extern float    real_vref;            /* Measured VDDA via VREFINT, V */

/* ========================================================================= */
/*  SYSTEM HARDWARE DIAGNOSTIC BITMASKS                                      */
/* ========================================================================= */
#define _B_TEST_HARDWARE_SUCCESS_   0x00000000U
#define _B_FAULT_OS_                B0  /* System OS startup fault */
#define _B_FAULT_PINS_              B1  /* Pin configuration fault */
#define _B_FAULT_TERMINAL_          B2  /* Terminal / Host UART fault */
#define _B_FAULT_CFG_               B3  /* Configuration parameters fault */
#define _B_FAULT_BMP280_            B4  /* BMP280 Sensor fault */
#define _B_FAULT_FPGA_              B5  /* Spartan-6 boot / SPI communication fault */
#define _B_FAULT_HOST_UART_         B6  /* BC UART2 communication fault */

/* Non-fatal POST faults: logged for diagnostics but MUST NOT stop the
   400 Hz real-time control loop. BMP280 (barometric) is auxiliary telemetry
   only: a dead sensor must never take the whole weapon system offline. */
#define _B_NONFATAL_FAULTS_         _B_FAULT_BMP280_

/* ========================================================================= */
/*  GLOBAL BSP FUNCTION PROTOTYPES                                           */
/* ========================================================================= */

/* Power-On Self Test and System APIs */
uint32_t	init_hardware(void);
bool		test_status_hardware(uint32_t module);
uint32_t	get_status_hardware(void);
bool		get_rcc_csr(void);
void		bsp_system_reset(void);

/* Transceiver Direction Control */
void		ten(bool par);

/* Internal Temperature Measurements */
float		Read_Temperature(void);

/* DWT Microsecond Blocking Delay */
void		DWT_Init(void);
void		DWT_DeInit(void);
void		delay_us(const uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_SUPPORT_PACKAGE_H */
