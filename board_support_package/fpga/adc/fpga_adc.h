/**
 ******************************************************************************
 * @file    fpga_adc.h
 * @brief   Direct ADC Telemetry Extraction & Configurable Scaling for PSPM.
 *          Target MCU: STM32F411 (Cortex-M4 @ 100 MHz).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef FPGA_ADC_H
#define FPGA_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "board_support_package.h"
#include "fpga_reg_map.h"
#include "usefull_define.h"

/* ========================================================================= */
/*  LOCK-IN SCALE FACTORS                                                    */
/* ========================================================================= */

/* Peak Amplitude Scaling: V_peak = (Accumulator / Count) * (PI / 2.0) */
/* RMS reading (if ever required): V_rms = V_peak / sqrt(2) */
#define AC_SCALE_PEAK                1.570796327f

/* ========================================================================= */
/*  EXACT LSB SENSITIVITIES FROM TECHNICAL SPECIFICATION                     */
/* ========================================================================= */

/* 1 LSB = 400 arcmin / 32768 = 0.01220703125 arcmin */
#define LSB_ERR_EL_ARCMIN            (400.0f / 32768.0f)

/* 1 LSB = 40 arcmin / 32768 = 0.001220703125 arcmin */
#define LSB_ERR_EL_10_ARCMIN         (40.0f  / 32768.0f)

/* 1 LSB = 400 arcmin / 32768 = 0.01220703125 arcmin */
#define LSB_ERR_AZ_ARCMIN            (400.0f / 32768.0f)

/* 1 LSB = 40 arcmin / 32768 = 0.001220703125 arcmin */
#define LSB_ERR_AZ_10_ARCMIN         (40.0f  / 32768.0f)

/* 1 LSB = 30 deg / 65536 = 0.00045776367 deg */
#define LSB_GUN_POS_DEG              (30.0f  / 65536.0f)

/* 1 LSB = 40 m/s / 65536 = 0.00061035156 m/s */
#define LSB_V_TANK_MPS               (40.0f  / 65536.0f)

/* 1 LSB = 50 m/s / 65536 = 0.00076293945 m/s */
#define LSB_V_WIND_MPS               (50.0f  / 65536.0f)

/* 1 LSB = 2.0 / 65536 = 0.000030517578 */
#define LSB_COS_FI                   (2.0f   / 65536.0f)

/* 1 LSB = 10.0V / 32768 = 0.00030517578 V */
#define LSB_VOLTAGE_10V              (10.0f  / 32768.0f)

/* ========================================================================= */
/*  DATA STRUCTURES                                                          */
/* ========================================================================= */

/**
 * @brief  ADC Telemetry Container with Dual Representation (16-bit + Float)
 * @note   Channel semantics follow the FPGA analog input map (FPGA is truth):
 *         AD7367 conversion-pair interleave inside every chip:
 *         ch1 = A1, ch2 = B1, ch3 = A2, ch4 = B2.
 */
typedef struct {
    /* --- 1. DIRECT 16-BIT INTEGER VIEW (For PSPM -> BC Packet) --- */
    struct {
        // DA21 Elevation 4AC (sync UON23): A1=ERR_EL, B1=ERR_EL1O, A2=W_EL, B2=GUN_POS
        int16_t err_el;       /* ac1 = ERR_EL    : D[0..1]   (+-400 arcmin)                    */
        int16_t err_el_10;    /* ac2 = ERR_EL1O  : D[2..3]   (+-40 arcmin)                     */
        int16_t w_el_in;      /* ac3 = W_EL rate input (aux; packet W_EL uses resolver)        */
        int16_t gun_pos;      /* ac4 = GUN_POS   : D[12..13] (-15..+15 deg, amplitude source)  */

        // DA22 Azimuth 4AC (sync UON12): A1=ERR_AZ, B1=ERR_AZ1O, A2=RESERVE, B2=W_AZ
        int16_t err_az;       /* ac1 = ERR_AZ    : D[4..5]   (+-400 arcmin)                    */
        int16_t err_az_10;    /* ac2 = ERR_AZ1O  : D[6..7]   (+-40 arcmin)                     */
        int16_t da22_reserve; /* ac3 = RESERVE (VON12 monitor)                                 */
        int16_t w_az_in;      /* ac4 = W_AZ rate input (aux; packet W_AZ uses resolver)        */

        // DA45 Telemetry 4DC: A1=SPEED_TANK, B1=COS_FI, A2=SPEED_WIND, B2=RESERVE
        int16_t v_tank;       /* dc1 = SPEED_TANK : D[18..19] (-20..+20 m/s)                   */
        int16_t cos_fi;       /* dc2 = COS_FI     : D[22..23] (-1.0..+1.0)                     */
        int16_t v_wind;       /* dc3 = SPEED_WIND : D[20..21] (-25..+25 m/s)                   */
        int16_t da45_reserve; /* dc4 = RESERVE    : unused                                     */
    } raw_16b;

    /* --- 2. PHYSICAL FLOAT VIEW (For DSPA Terminal / GUI Display) --- */
    struct {
        // DA21 Elevation
        float err_el_arcmin;     /* Vertical Error in arcminutes                    */
        float err_el_10_arcmin;  /* Vertical Error x10 in arcminutes                */
        float w_el_in_volt;      /* W_EL analog input amplitude [V] (aux)           */
        float gun_pos_deg;       /* Gun Position in degrees (-15.0..+15.0 deg)      */

        // DA22 Azimuth
        float err_az_arcmin;     /* Azimuth Error in arcminutes                     */
        float err_az_10_arcmin;  /* Azimuth Error x10 in arcminutes                 */
        float da22_reserve_volt; /* RESERVE input voltage [V]                       */
        float w_az_in_volt;      /* W_AZ analog input amplitude [V] (aux)           */

        // DA45 Telemetry
        float v_tank_mps;        /* Tank Speed in m/s (-20.0..+20.0 m/s)            */
        float v_wind_mps;        /* Crosswind Speed in m/s (-25.0..+25.0 m/s)       */
        float cos_fi;            /* Turret Rotation Cosine (-1.0..+1.0)             */
    } phys;

    bool is_valid;
} fpga_adc_telemetry_t;

extern fpga_adc_telemetry_t g_fpga_adc_telemetry;

/* ========================================================================= */
/*  PUBLIC API PROTOTYPES                                                    */
/* ========================================================================= */

void                        FPGA_ADC_Init(void);
void                        FPGA_ADC_Process(const FpgaDmaTelemetryPacket_t *raw_dma);
const fpga_adc_telemetry_t* FPGA_ADC_GetTelemetry(void);

#ifdef __cplusplus
}
#endif

#endif /* FPGA_ADC_H */
