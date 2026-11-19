/**
 ******************************************************************************
 * @file    pin_mgmt.h
 * @brief   Header for STM32 Pin Management & Dual-Plane Bus Arbitration Driver.
 *          Target MCU: STM32F411RET6 (100 MHz, FreeRTOS CMSIS-OS v2).
 *          Board: PSPM (DIAS.901.02.00.300).
 *          All comments in pure ASCII English.
 ******************************************************************************
 */

#ifndef __PIN_MGMT_H
#define __PIN_MGMT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os2.h"
#include "gpio_fallbacks.h"

/* ========================================================================= */
/*  DATA STRUCTURES & ENUMS                                                  */
/* ========================================================================= */

/**
 * @brief Pin descriptor structure linking GPIO port, pin mask, and name.
 */
typedef struct {
    GPIO_TypeDef* port;          /*!< GPIO Port (GPIOA, GPIOB, etc.) */
    uint16_t pin;                /*!< Pin Mask (GPIO_PIN_0, etc.) */
    const char* name;            /*!< Pin Name for diagnostics */
} Pin_Descriptor_t;

/**
 * @brief SPI1 physical bus operational modes
 */
typedef enum {
    SPI_BUS_AF_MODE = 0,         /*!< Active SPI1 Master mode (AF5) for STM32 */
    SPI_BUS_HIZ_MODE             /*!< Tri-State (Analog mode) during FPGA power-up configuration */
} SPI_Bus_Mode_t;

typedef struct {
    uint8_t debug_enabled;
} Pin_Mgmt_Config_t;

#define PIN_MGMT_CONFIG_DEFAULT { .debug_enabled = 0 }

#define PIN_DESC(_port, _pin, _name) \
    { .port = _port, .pin = _pin, .name = _name }

extern Pin_Mgmt_Config_t bsp_pin_config;

/* ========================================================================= */
/*  PUBLIC API PROTOTYPES                                                    */
/* ========================================================================= */

/* Subsystem Lifecycle */
osStatus_t  PIN_MGMT_Init(Pin_Mgmt_Config_t *config);
osStatus_t  PIN_MGMT_DeInit(void);

/* Standard Mutex-Protected Operations */
osStatus_t  PIN_Set(const Pin_Descriptor_t *pin);
osStatus_t  PIN_Reset(const Pin_Descriptor_t *pin);
osStatus_t  PIN_Toggle(const Pin_Descriptor_t *pin);
uint8_t     PIN_Read(const Pin_Descriptor_t *pin);

/* Fast Atomic Register Access (BSRR / IDR - Zero Mutex Overhead) */
osStatus_t  PIN_Set_F(const Pin_Descriptor_t *pin);
osStatus_t  PIN_Reset_F(const Pin_Descriptor_t *pin);
osStatus_t  PIN_Toggle_F(const Pin_Descriptor_t *pin);
uint8_t     PIN_Read_F(const Pin_Descriptor_t *pin);

/* GPIO Mutex Controls */
osStatus_t  PIN_GPIO_Mutex_Acquire(uint32_t timeout);
osStatus_t  PIN_GPIO_Mutex_Release(void);
uint8_t     PIN_GPIO_Mutex_Is_Locked(void);

/* FPGA Bring-up & Status Synchronization */
osStatus_t  FPGA_Reset_With_Check(uint32_t reset_time_ms, uint32_t timeout_ms);
osStatus_t  FPGA_Wait_Ready(uint32_t timeout_ms);
uint8_t     FPGA_Is_Ready(void);
uint8_t     FPGA_Is_In_Reset(void);

/* SPI1 Bus Mode Arbitration */
osStatus_t  SPI_Bus_Configure(SPI_Bus_Mode_t mode);
osStatus_t  SPI_Bus_Acquire_For_STM32(void);
osStatus_t  SPI_Bus_Release_To_FPGA(void);
uint8_t     SPI_Bus_Is_Acquired(void);

/* Diagnostics & Utilities */
uint8_t     STLINK_Is_Connected(void);
osStatus_t  PIN_Delay(uint32_t ms);
osStatus_t  PIN_Blink(const Pin_Descriptor_t *pin, uint8_t count, uint32_t delay_ms);

/* General convenience macros */
#define PIN_ON(pin)          PIN_Set(pin)
#define PIN_OFF(pin)         PIN_Reset(pin)
#define PIN_TOGGLE(pin)      PIN_Toggle(pin)
#define PIN_STATE(pin)       PIN_Read(pin)

#define PIN_ON_F(pin)        PIN_Set_F(pin)
#define PIN_OFF_F(pin)       PIN_Reset_F(pin)
#define PIN_TOGGLE_F(pin)    PIN_Toggle_F(pin)
#define PIN_STATE_F(pin)     PIN_Read_F(pin)

/* ========================================================================= */
/*  HARDWARE PIN DESCRIPTORS DECLARATIONS                                    */
/* ========================================================================= */
extern const Pin_Descriptor_t pin_led_green;

/* extern const Pin_Descriptor_t pin_clk25mhz;  FORBIDDEN: PH0 = HSE 25 MHz in,
   never touch as GPIO (PLL runs from this clock). */
extern const Pin_Descriptor_t pin_tp1;
extern const Pin_Descriptor_t pin_tp2;
extern const Pin_Descriptor_t pin_tp3;
extern const Pin_Descriptor_t pin_tp4;
extern const Pin_Descriptor_t pin_fpga_done;
extern const Pin_Descriptor_t pin_usart2_fpga_tx;
extern const Pin_Descriptor_t pin_usart2_fpga_rx;
extern const Pin_Descriptor_t pin_spi_sck;
extern const Pin_Descriptor_t pin_spi_miso;
extern const Pin_Descriptor_t pin_spi_mosi;

/* Distinct Dual-Plane Chip Select Pins for PSPM */
extern const Pin_Descriptor_t pin_flash_cs;           /* PA4: Flash W25Q128 CS */
extern const Pin_Descriptor_t pin_stm32_2_fpga_nss_p; /* PB0: FPGA Polling CS (Control Plane) */
extern const Pin_Descriptor_t pin_fpga_cs;            /* Alias -> PB0 */
extern const Pin_Descriptor_t pin_stm32_2_fpga_nss_d; /* PB2: FPGA DMA CS (Data Plane) */
extern const Pin_Descriptor_t pin_interrupt_fpga;     /* PB10: FPGA IRQ (EXTI10) */

/* Bosch BMP280 Dedicated SPI3 Chip Select */
extern const Pin_Descriptor_t pin_spi3_cso;           /* PC10: BMP280 CS */

/* Status and Reset Pins */
extern const Pin_Descriptor_t pin_fpga_2_stm32_misc1;
extern const Pin_Descriptor_t pin_mr_fpga;
extern const Pin_Descriptor_t pin_mr_prog;           /* PB15: PROG_B */
extern const Pin_Descriptor_t pin_usart6_debug_tx;
extern const Pin_Descriptor_t pin_usart6_debug_rx;
extern const Pin_Descriptor_t pin_usart1_kpa_te;
extern const Pin_Descriptor_t pin_usart1_kpa_tx;
extern const Pin_Descriptor_t pin_usart1_kpa_rx;
extern const Pin_Descriptor_t pin_stlink_detect;

#ifdef __cplusplus
}
#endif

#endif /* __PIN_MGMT_H */
