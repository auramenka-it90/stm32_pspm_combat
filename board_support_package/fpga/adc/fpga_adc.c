/**
 ******************************************************************************
 * @file    fpga_adc.c
 * @brief   Direct ADC Telemetry Extraction & Fixed Peak (PI/2) Scaling.
 *          Target MCU: STM32F411 (Cortex-M4 @ 100 MHz).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "fpga_adc.h"
#include <string.h>

/* Global physical telemetry storage */
fpga_adc_telemetry_t g_fpga_adc_telemetry = {0};

/* ========================================================================= */
/*  PUBLIC API IMPLEMENTATION                                                */
/* ========================================================================= */

void FPGA_ADC_Init(void) {
    memset(&g_fpga_adc_telemetry, 0, sizeof(fpga_adc_telemetry_t));
}

void FPGA_ADC_Process(const FpgaDmaTelemetryPacket_t *raw_dma) {
    if (raw_dma == NULL) {
        g_fpga_adc_telemetry.is_valid = false;
        return;
    }

    // =========================================================================
    // 1. ELEVATION CHANNELS: DA21 (4x AC Channels Synced to UON23)
    //    FPGA map: ac1=A1 ERR_EL, ac2=B1 ERR_EL1O, ac3=A2 W_EL, ac4=B2 GUN_POS
    // =========================================================================
    if (raw_dma->da21_ac_cnt > 0) {
        float inv_cnt_da21 = 1.0f / (float)raw_dma->da21_ac_cnt;

        // Peak scaling: rectified average -> sine amplitude, factor PI/2
        float scale_da21 = AC_SCALE_PEAK * inv_cnt_da21;

        // Raw 16-bit signed scaled integers
        g_fpga_adc_telemetry.raw_16b.err_el    = (int16_t)((float)raw_dma->da21_ac1 * scale_da21);
        g_fpga_adc_telemetry.raw_16b.err_el_10 = (int16_t)((float)raw_dma->da21_ac2 * scale_da21);
        g_fpga_adc_telemetry.raw_16b.w_el_in   = (int16_t)((float)raw_dma->da21_ac3 * scale_da21);
        g_fpga_adc_telemetry.raw_16b.gun_pos   = (int16_t)((float)raw_dma->da21_ac4 * scale_da21);
    } else {
        g_fpga_adc_telemetry.raw_16b.err_el    = 0;
        g_fpga_adc_telemetry.raw_16b.err_el_10 = 0;
        g_fpga_adc_telemetry.raw_16b.w_el_in   = 0;
        g_fpga_adc_telemetry.raw_16b.gun_pos   = 0;
    }

    // Calculate physical arcminutes, volts and degrees
    // GUN_POS is an AC transmitter amplitude: LSB_GUN_POS_DEG maps full-scale
    // amplitude to +-15 deg; verify/trim the gain on the bench.
    g_fpga_adc_telemetry.phys.err_el_arcmin    = (float)g_fpga_adc_telemetry.raw_16b.err_el * LSB_ERR_EL_ARCMIN;
    g_fpga_adc_telemetry.phys.err_el_10_arcmin = (float)g_fpga_adc_telemetry.raw_16b.err_el_10 * LSB_ERR_EL_10_ARCMIN;
    g_fpga_adc_telemetry.phys.w_el_in_volt     = (float)g_fpga_adc_telemetry.raw_16b.w_el_in * LSB_VOLTAGE_10V;
    g_fpga_adc_telemetry.phys.w_el_dps         = (float)g_fpga_adc_telemetry.raw_16b.w_el_in * LSB_W_EL_DPS;
    g_fpga_adc_telemetry.phys.gun_pos_deg      = (float)g_fpga_adc_telemetry.raw_16b.gun_pos * LSB_GUN_POS_DEG;

    // =========================================================================
    // 2. AZIMUTH CHANNELS: DA22 (4x AC Channels Synced to UON12)
    //    FPGA map: ac1=A1 ERR_AZ, ac2=B1 ERR_AZ1O, ac3=A2 RESERVE, ac4=B2 W_AZ
    // =========================================================================
    if (raw_dma->da22_ac_cnt > 0) {
        float inv_cnt_da22 = 1.0f / (float)raw_dma->da22_ac_cnt;

        // Peak scaling: rectified average -> sine amplitude, factor PI/2
        float scale_da22 = AC_SCALE_PEAK * inv_cnt_da22;

        // Raw 16-bit signed scaled integers
        g_fpga_adc_telemetry.raw_16b.err_az       = (int16_t)((float)raw_dma->da22_ac1 * scale_da22);
        g_fpga_adc_telemetry.raw_16b.err_az_10    = (int16_t)((float)raw_dma->da22_ac2 * scale_da22);
        g_fpga_adc_telemetry.raw_16b.da22_reserve = (int16_t)((float)raw_dma->da22_ac3 * scale_da22);
        g_fpga_adc_telemetry.raw_16b.w_az_in      = (int16_t)((float)raw_dma->da22_ac4 * scale_da22);
    } else {
        g_fpga_adc_telemetry.raw_16b.err_az       = 0;
        g_fpga_adc_telemetry.raw_16b.err_az_10    = 0;
        g_fpga_adc_telemetry.raw_16b.da22_reserve = 0;
        g_fpga_adc_telemetry.raw_16b.w_az_in      = 0;
    }

    // Calculate physical arcminutes & voltages
    g_fpga_adc_telemetry.phys.err_az_arcmin     = (float)g_fpga_adc_telemetry.raw_16b.err_az * LSB_ERR_AZ_ARCMIN;
    g_fpga_adc_telemetry.phys.err_az_10_arcmin  = (float)g_fpga_adc_telemetry.raw_16b.err_az_10 * LSB_ERR_AZ_10_ARCMIN;
    g_fpga_adc_telemetry.phys.da22_reserve_volt = (float)g_fpga_adc_telemetry.raw_16b.da22_reserve * LSB_VOLTAGE_10V;
    g_fpga_adc_telemetry.phys.w_az_in_volt      = (float)g_fpga_adc_telemetry.raw_16b.w_az_in * LSB_VOLTAGE_10V;
    g_fpga_adc_telemetry.phys.w_az_dps          = (float)g_fpga_adc_telemetry.raw_16b.w_az_in * LSB_W_AZ_DPS;

    // =========================================================================
    // 3. SENSOR TELEMETRY CHANNELS: DA45 (4x Decimated DC Channels from FPGA)
    //    FPGA map (AD7367 A/B interleave): dc1=A1 SPEED_TANK, dc2=B1 COS_FI,
    //    dc3=A2 SPEED_WIND, dc4=B2 RESERVE. GUN_POS is NOT on DA45 (it is the
    //    DA21 AC4 amplitude channel, handled above).
    // =========================================================================
    g_fpga_adc_telemetry.raw_16b.v_tank       = raw_dma->da45_dc1; // D[18..19]
    g_fpga_adc_telemetry.raw_16b.cos_fi       = raw_dma->da45_dc2; // D[22..23]
    g_fpga_adc_telemetry.raw_16b.v_wind       = raw_dma->da45_dc3; // D[20..21]
    g_fpga_adc_telemetry.raw_16b.da45_reserve = raw_dma->da45_dc4; // unused

    // Calculate physical engineering units for Terminal
    g_fpga_adc_telemetry.phys.v_tank_mps = (float)g_fpga_adc_telemetry.raw_16b.v_tank * LSB_V_TANK_MPS;
    g_fpga_adc_telemetry.phys.v_wind_mps = (float)g_fpga_adc_telemetry.raw_16b.v_wind * LSB_V_WIND_MPS;
    g_fpga_adc_telemetry.phys.cos_fi     = (float)g_fpga_adc_telemetry.raw_16b.cos_fi * LSB_COS_FI;

    g_fpga_adc_telemetry.is_valid = true;
}

const fpga_adc_telemetry_t* FPGA_ADC_GetTelemetry(void) {
    return &g_fpga_adc_telemetry;
}
