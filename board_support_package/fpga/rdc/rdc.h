/**
 ******************************************************************************
 * @file    rdc.h
 * @brief   Unified Resolver-to-Digital Converter (RDC) Subsystem for PSPM.
 *          Implements Type-II Tracking Phase-Locked Loop (PLL) DSP Observer.
 *          Target MCU: STM32F411 (Cortex-M4 @ 100 MHz, FreeRTOS).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef RDC_H
#define RDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "board_support_package.h"
#include "fpga_reg_map.h"
#include "stm32_2_fpga_spi_bridge.h"
#include "usefull_define.h"

/* ========================================================================= */
/*  MATHEMATICAL CORE CONFIGURATION DEFAULTS & LIMITS                        */
/* ========================================================================= */

#define RDC_DEFAULT_CARRIER_FREQ_HZ     400.0f  /* 400 Hz Carrier frequency (fixed by FPGA sampling) */
#define RDC_DEFAULT_LOOP_BANDWIDTH_HZ   15.0f   /* 15 Hz Tracking Loop Bandwidth */
#define RDC_DEFAULT_DAMPING_RATIO       0.707f  /* Optimal Butterworth damping (zeta = 1/sqrt(2)) */
#define RDC_DEFAULT_MAX_SPEED_DPS       1000.0f /* Anti-windup velocity clamp: 1000 deg/s */
#define RDC_DEFAULT_ADD_PI              true    /* Add 180 deg offset to BAM16 angle */

/* Validation Limits (enforced on every terminal edit, save_setting, RDC_Init) */
#define RDC_MIN_CARRIER_FREQ_HZ         375.0f  /* FPGA comparator window: 400 Hz -6.25% */
#define RDC_MAX_CARRIER_FREQ_HZ         425.0f  /* FPGA comparator window: 400 Hz +6.25% */
#define RDC_MIN_LOOP_BW_HZ              0.1f    /* Lower bound; upper bound is dynamic: carrier/2 (Nyquist) */
#define RDC_MIN_DAMPING_RATIO           0.1f    /* Below: numerically twitchy underdamped loop */
#define RDC_MAX_DAMPING_RATIO           5.0f    /* Above: overdamped, loop stops tracking */
#define RDC_MIN_SPEED_DPS               0.0f    /* Exclusive lower bound (speed limit must be > 0) */
#define RDC_MAX_SPEED_DPS               1000.0f /* Physical ceiling of the airborne resolver */

/* Hardware Signal Quality Thresholds */
#define RDC_MIN_AMPLITUDE_THRESHOLD     10000.0f /* Broken cable detection threshold */

/* Reference Carrier Frequency Watchdog Limits (100 kHz Sample Rate) */
#define RDC_ADC_SAMPLE_RATE_HZ          100000.0f
#define RDC_MIN_VALID_FREQ_HZ           200.0f   /* 200 Hz lower carrier limit */
#define RDC_MAX_VALID_FREQ_HZ           600.0f   /* 600 Hz upper carrier limit */

#define RDC_MAX_SAMPLES_LIMIT           (uint16_t)(RDC_ADC_SAMPLE_RATE_HZ / RDC_MIN_VALID_FREQ_HZ) /* ~500 ticks */
#define RDC_MIN_SAMPLES_LIMIT           (uint16_t)(RDC_ADC_SAMPLE_RATE_HZ / RDC_MAX_VALID_FREQ_HZ) /* ~166 ticks */

/* ========================================================================= */
/*  HARDWARE ADC CHANNEL ROUTING MATRIX (AD7367 / FPGA Device ID 4)          */
/* ========================================================================= */

#define AD7367_CH_VA1                   0x00U /* A1: SIN_ROLL  (roll axis sine)      */
#define AD7367_CH_VB1                   0x01U /* B1: SIN_PITCH (pitch axis sine)     */
#define AD7367_CH_VA2                   0x02U /* A2: COS_ROLL  (roll axis cosine)    */
#define AD7367_CH_VB2                   0x03U /* B2: COS_PITCH (pitch axis cosine)   */

#define RDC_SEL_SIN_ROLL_SHIFT          0U
#define RDC_SEL_SIN_PITCH_SHIFT         2U
#define RDC_SEL_COS_ROLL_SHIFT          4U
#define RDC_SEL_COS_PITCH_SHIFT         6U

/* Routing per board analog map, identical to the FPGA reset default 8'hE4
   (selector encoding: 00=A1, 01=B1, 10=A2, 11=B2 per adc_processor_resolver) */
#define RDC_DEFAULT_ROUTING_MASK        ((AD7367_CH_VA1 << RDC_SEL_SIN_ROLL_SHIFT)  | \
                                         (AD7367_CH_VB1 << RDC_SEL_SIN_PITCH_SHIFT) | \
                                         (AD7367_CH_VA2 << RDC_SEL_COS_ROLL_SHIFT)  | \
                                         (AD7367_CH_VB2 << RDC_SEL_COS_PITCH_SHIFT))

/* ========================================================================= */
/*  DATA STRUCTURES                                                          */
/* ========================================================================= */

/**
 * @brief Type-II PLL Tracking Loop Parameters Configuration
 */
typedef struct {
    float carrier_freq_hz;     /* Carrier frequency (Hz, default: 400.0f) */
    float loop_bandwidth_hz;   /* Tracking loop bandwidth (Hz, default: 15.0f) */
    float damping_ratio;       /* Damping ratio (default: 0.707f) */
    float max_speed_dps;       /* Anti-windup velocity limit (deg/s, default: 1000.0f) */
    bool  add_PI;              /* Add 180 deg offset to BAM16 angle */
} rdc_math_cfg_t;

/**
 * @brief Complete Resolver Processed Telemetry Output
 */
typedef struct {
    /* Raw Hardware Integrals (Extracted from 34-word DMA packet) */
    int32_t  sin_roll;
    int32_t  cos_roll;
    int32_t  sin_pitch;
    int32_t  cos_pitch;
    uint16_t samples_cnt;
    bool     is_valid;

    /* Roll Channel Physical Results */
    float    roll_angle_rad;   /* Roll Angle in Radians [-PI, +PI] */
    float    roll_angle_deg;   /* Roll Angle in Degrees [-180.0, +180.0] */
    float    roll_speed_dps;   /* Roll Angular Velocity in deg/s */
    int16_t  roll_angle_bf;    /* Roll Angle in BAM16 format (-32768..+32767) */
    float    roll_error;       /* Phase detector tracking error */

    /* Pitch Channel Physical Results */
    float    pitch_angle_rad;  /* Pitch Angle in Radians [-PI, +PI] */
    float    pitch_angle_deg;  /* Pitch Angle in Degrees [-180.0, +180.0] */
    float    pitch_speed_dps;  /* Pitch Angular Velocity in deg/s */
    int16_t  pitch_angle_bf;   /* Pitch Angle in BAM16 format (-32768..+32767) */
    float    pitch_error;      /* Phase detector tracking error */
} rdc_telemetry_t;

extern rdc_math_cfg_t   g_rdc_math_cfg;
extern rdc_telemetry_t  g_rdc_telemetry;

/* ========================================================================= */
/*  PUBLIC API PROTOTYPES                                                    */
/* ========================================================================= */

/**
 * @brief  Initializes Type-II PLL tracking observers for both Roll and Pitch axes.
 * @param  cfg: Pointer to configuration (or NULL to load factory defaults).
 */
void                   RDC_Init(const rdc_math_cfg_t *cfg);

/**
 * @brief  Validates tracking loop configuration parameters.
 */
bool                   RDC_ValidateConfig(const rdc_math_cfg_t *cfg);

/**
 * @brief  Main 400 Hz real-time DSP calculation engine.
 *         Extracts raw demodulated integrals from DMA packet, executes Type-II
 *         PLL observer, calculates angles, speeds, and BAM16 representation.
 * @param  raw_dma: Pointer to incoming 34-word DMA telemetry packet.
 */
void                   RDC_Process(const FpgaDmaTelemetryPacket_t *raw_dma);

/**
 * @brief  Configures hardware ADC input crossbar routing matrix in FPGA (Device ID = 4).
 */
FPGA_Status_t          RDC_SetRouting(FPGA_HandleTypeDef *hbridge, uint8_t sel_mask);

/**
 * @brief  Returns pointer to the latest processed resolver telemetry.
 */
const rdc_telemetry_t* RDC_GetTelemetry(void);

#ifdef __cplusplus
}
#endif

#endif /* RDC_H */
