/**
 ******************************************************************************
 * @file    bmp280.h
 * @brief   High-Precision Driver for Bosch BMP280 Pressure & Temperature Sensor.
 *          Target: SPI Master Mode (SPI3 @ 6.25 MHz, 8-bit, Mode 00).
 *          Dedicated SPI3 bus (SCK PB12/AF7, MISO PC11, MOSI PC12, CS PC10),
 *          independent from the FPGA SPI1 bus.
 *          Datasheet reference: BST-BMP280-DS001-10 (rev 1.13).
 *          All comments in English.
 ******************************************************************************
 */

#ifndef BMP280_H
#define BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pin_mgmt.h"
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================= */
/*  BMP280 REGISTER ADDRESS MAP & COMMANDS (Bosch Datasheet Table 18)        */
/* ========================================================================= */
#define BMP280_REG_CALIB00           0x88U /* Calibration data calib00..calib25 (0x88..0xA1) */
#define BMP280_REG_ID                0xD0U /* Chip identification register */
#define BMP280_REG_RESET             0xE0U /* Soft reset register */
#define BMP280_REG_STATUS            0xF3U /* Device status register (measuring, im_update) */
#define BMP280_REG_CTRL_MEAS         0xF4U /* Measurement control (osrs_t, osrs_p, mode) */
#define BMP280_REG_CONFIG            0xF5U /* Configuration register (t_sb, filter, spi3w_en) */
#define BMP280_REG_PRESS_MSB         0xF7U /* Pressure MSB [19:12] */
#define BMP280_REG_PRESS_LSB         0xF8U /* Pressure LSB [11:4] */
#define BMP280_REG_PRESS_XLSB        0xF9U /* Pressure XLSB [3:0] */
#define BMP280_REG_TEMP_MSB          0xFAU /* Temperature MSB [19:12] */
#define BMP280_REG_TEMP_LSB          0xFBU /* Temperature LSB [11:4] */
#define BMP280_REG_TEMP_XLSB         0xFCU /* Temperature XLSB [3:0] */

/* Chip Identification Values */
#define BMP280_CHIP_ID_VAL           0x58U /* Standard Bosch BMP280 ID */
#define BME280_CHIP_ID_VAL           0x60U /* Bosch BME280 Combined Sensor ID */

/* ========================================================================= */
/*  CONTROL REGISTER BIT LAYOUT (Bosch Datasheet Table 20 / 21)              */
/*  ctrl_meas (0xF4): [7:5] osrs_t, [4:2] osrs_p, [1:0] mode                 */
/*    osrs: 000=skipped 001=x1 010=x2 011=x4 100=x8 101=x16                   */
/*    mode: 00=sleep  01/10=forced  11=normal (auto-cycling with t_sb)        */
/*  config  (0xF5): [7:5] t_sb, [4:2] filter, [0] spi3w_en                    */
/*    t_sb:    000=0.5 001=62.5 010=125 011=250 ... ms standby                */
/*    filter:  000=off 001=x2 010=x4 011=x8 100..=x16 IIR                     */
/*  Values written by BMP280_Init (Bosch "handheld low-power" ~10 Hz profile):*/
/*    config    = 0x30 -> t_sb=001 (62.5 ms), filter=100 (x16)                */
/*    ctrl_meas = 0x57 -> osrs_t=010 (x2), osrs_p=101 (x16), mode=11 (normal) */
/* ========================================================================= */



/* Factory NVM calibration coefficients, read once at init (datasheet Table 17).
   T* are temperature trim, P* are pressure trim coefficients consumed by the
   Bosch compensation formulas. dig_T1/dig_P1 are unsigned, the rest signed. */
typedef struct {
    uint16_t dig_T1;  /* 0x88/0x89 */
    int16_t  dig_T2;  /* 0x8A/0x8B */
    int16_t  dig_T3;  /* 0x8C/0x8D */
    uint16_t dig_P1;  /* 0x8E/0x8F */
    int16_t  dig_P2;  /* 0x90/0x91 */
    int16_t  dig_P3;  /* 0x92/0x93 */
    int16_t  dig_P4;  /* 0x94/0x95 */
    int16_t  dig_P5;  /* 0x96/0x97 */
    int16_t  dig_P6;  /* 0x98/0x99 */
    int16_t  dig_P7;  /* 0x9A/0x9B */
    int16_t  dig_P8;  /* 0x9C/0x9D */
    int16_t  dig_P9;  /* 0x9E/0x9F */
} BMP280_Calib_t;


/* Soft Reset Command (write 0xB6 to register 0xE0 to trigger POR sequence) */
#define BMP280_CMD_SOFT_RESET        0xB6U

/* Standard Atmosphere (ISA) sea-level reference for the altitude formula */
#define BMP280_SEA_LEVEL_PRESSURE_PA 101325.0f

/* Status codes returned by the driver API */
typedef enum {
    BMP280_OK       = 0,   /* Operation successful                              */
    BMP280_ERROR    = 1,   /* SPI transfer / argument error                     */
    BMP280_ID_ERROR = 2    /* Chip ID mismatch: sensor absent or not a BMP280   */
} BMP280_Status_t;

/* Telemetry Data Container */
typedef struct {
    float temperature_c; /* Temperature in degrees Celsius (degC) */
    float pressure_pa;   /* Atmospheric pressure in Pascals (Pa) */
    float pressure_mmhg; /* Atmospheric pressure in mmHg (BC packet unit) */
    float altitude_m;    /* Barometric altitude in meters (m) */
    bool  is_valid;      /* Data validity flag */
} BMP280_Data_t;

/* Global telemetry snapshot (written by BMP280_Process @10Hz, read by BC
   packet builder and DSPA terminal signals), same pattern as g_rdc_telemetry */
extern BMP280_Data_t g_bmp280_telemetry;

/* Chip presence flag: true after a successful chip-ID match in BMP280_Init,
   kept truthful at runtime by the 1 Hz ID watchdog in BMP280_Process() */
extern bool g_bmp280_present;

/* ========================================================================= */
/*  PUBLIC API PROTOTYPES                                                    */
/* ========================================================================= */

BMP280_Status_t BMP280_Init(SPI_HandleTypeDef *hspi, const Pin_Descriptor_t *cs_pin);
BMP280_Status_t BMP280_Process(void);
bool            BMP280_IsConnected(void);

/* Data accessors.
   BMP280_GetTelemetry is the validity-gated snapshot API: it returns false
   (and leaves *out_data untouched) when no fresh valid sample exists, so the
   BC packet builder can report "no data" instead of a stale value.
   The scalar Getters below return the last cached numbers regardless of
   validity: use them for diagnostics only, never for the BC protocol field. */
bool            BMP280_GetTelemetry(BMP280_Data_t *out_data);
float           BMP280_GetTemperature(void);
float           BMP280_GetPressure(void);
float           BMP280_GetAltitude(void);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
