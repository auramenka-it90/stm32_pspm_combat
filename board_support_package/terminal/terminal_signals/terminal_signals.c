/**
 ******************************************************************************
 * @file    terminal_signals.c
 * @brief   DSPA (eAssist) Terminal Telemetry Signals Tree for PSPM System.
 *          Read-only telemetry + RDC PLL config subtree with Apply&Save +
 *          DAC debug waveform generator subtree (sDBG).
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "dspa.h"
#include "dspa_defs.h"
#include "dspa_sigdefs.h"
#include "terminal_signals.h"
#include "fpga_control.h"
#include "ic.h"
#include "app_tasks.h"
#include <stddef.h>



/* Telemetry tree hierarchy node pointers */
static char *sDEV   = "";
static char *sCFG   = "";
static char *sRDC   = "";
static char *sADC   = "";
static char *sBMP   = "";
static char *sDEBUG = "";
static char *sDBG   = "";

/* Command Selector for RDC PLL Config Apply (1=Validate+Save Flash+RDC_Init, self-clearing) */
static uint8_t eAssist_rdc_apply_cmd = 0;

/* Formatted DMS angle strings (Degrees, Minutes, Seconds) */
static char str_roll_dms[64]  = " 0° 00' 00\"";
static char str_pitch_dms[64] = " 0° 00' 00\"";

/* ========================================================================= */
/*  HELPER: BAM16 TO DMS STRING CONVERSION (Zero-Jump Formatting)            */
/* ========================================================================= */
static void int16_to_dms(int16_t bam_val, char *out_buf, size_t buf_size) {
    float deg = BAM16_TO_DEG(bam_val);
    bool is_neg = (deg < 0.0f);
    if (is_neg) deg = -deg;

    // Round to nearest integer second (+0.5)
    int32_t total_sec = (int32_t)(deg * 3600.0f + 0.5f);

    int32_t d = total_sec / 3600;
    int32_t rem_sec = total_sec % 3600;
    int32_t m = rem_sec / 60;
    int32_t s = rem_sec % 60;

    // Constant 11-char width eliminates visual jitter around 0
    snprintf(out_buf, buf_size, "%s%ld° %02ld' %02ld\"",
             is_neg ? "-" : " ", (long)d, (long)m, (long)s);
}

void terminal_update_dynamic_strings(void) {
    int16_to_dms(g_rdc_telemetry.roll_angle_bf, str_roll_dms, sizeof(str_roll_dms));
    int16_to_dms(g_rdc_telemetry.pitch_angle_bf, str_pitch_dms, sizeof(str_pitch_dms));
}

/* ========================================================================= */
/*  DSPA TELEMETRY TREE DEFINITION (PSPM COMBAT)                             */
/* ========================================================================= */

SIGNALS_BEGIN(DSPA_SIGNALS_NAME)
	_STRING_R_  ("PSPM_Combat", sDEV, NULL),
		_U32_R_   ("Test hardware (0=OK)",        test_hardware_result,  &sDEV),
		_U64_R_   ("Serial number",               dev_cfg.item.serial,   &sDEV),

		/* --- CONFIGURATION & SETTINGS SUBTREE --- */
		_STRING_R_ ("Config (Sector 4)", sCFG, &sDEV),
			_FLOAT_RW_ ("RDC Carrier Freq [Hz]",  g_rdc_math_cfg.carrier_freq_hz,   &sCFG),
			_FLOAT_RW_ ("RDC Loop Bandwidth [Hz]", g_rdc_math_cfg.loop_bandwidth_hz, &sCFG),
			_FLOAT_RW_ ("RDC Damping Ratio",      g_rdc_math_cfg.damping_ratio,     &sCFG),
			_FLOAT_RW_ ("RDC Max Speed [deg/s]",  g_rdc_math_cfg.max_speed_dps,     &sCFG),
			_BOOL_RW_  ("RDC Add 180deg Offset",  g_rdc_math_cfg.add_PI,            &sCFG),
			_BYTE_RW_  ("RDC Apply&Save (1=Go)",  eAssist_rdc_apply_cmd,            &sCFG),

		/* --- RESOLVER (RDC TYPE-II PLL) SUBTREE --- */
		_STRING_R_ ("Resolver (RDC)", sRDC, &sDEV),
			_FLOAT_R_ ("Roll angle [°]",         g_rdc_telemetry.roll_angle_deg,  &sRDC),
			_FLOAT_R_ ("Pitch angle [°]",        g_rdc_telemetry.pitch_angle_deg, &sRDC),
			_FLOAT_R_ ("Roll speed [°/s]",       g_rdc_telemetry.roll_speed_dps,  &sRDC),
			_FLOAT_R_ ("Pitch speed [°/s]",      g_rdc_telemetry.pitch_speed_dps, &sRDC),
			_I16_R_   ("Roll BAM16",             g_rdc_telemetry.roll_angle_bf,   &sRDC),
			_I16_R_   ("Pitch BAM16",            g_rdc_telemetry.pitch_angle_bf,  &sRDC),
			_STRING_R_("Roll [DMS]",             str_roll_dms,                    &sRDC),
			_STRING_R_("Pitch [DMS]",            str_pitch_dms,                   &sRDC),
			_FLOAT_R_ ("Roll track error [rad]", g_rdc_telemetry.roll_error,      &sRDC),
			_FLOAT_R_ ("Pitch track error [rad]",g_rdc_telemetry.pitch_error,     &sRDC),
			_U16_R_   ("Sample count (400Hz)",   g_rdc_telemetry.samples_cnt,     &sRDC),
			_BOOL_R_  ("Carrier 400Hz valid",    g_rdc_telemetry.is_valid,        &sRDC),

		/* --- ADC SUBSYSTEM SUBTREE --- */
		_STRING_R_ ("ADC Subsystem", sADC, &sDEV),
			_FLOAT_R_("Err Elevation [arcmin]",    g_fpga_adc_telemetry.phys.err_el_arcmin,    &sADC),
			_FLOAT_R_("Err Elevation x10 [arcmin]",g_fpga_adc_telemetry.phys.err_el_10_arcmin, &sADC),
			_FLOAT_R_("Err Azimuth [arcmin]",      g_fpga_adc_telemetry.phys.err_az_arcmin,    &sADC),
			_FLOAT_R_("Err Azimuth x10 [arcmin]",  g_fpga_adc_telemetry.phys.err_az_10_arcmin, &sADC),
			_FLOAT_R_("Gun Position [°]",          g_fpga_adc_telemetry.phys.gun_pos_deg,      &sADC),
			_FLOAT_R_("Tank Speed [m/s]",          g_fpga_adc_telemetry.phys.v_tank_mps,       &sADC),
			_FLOAT_R_("Crosswind Speed [m/s]",     g_fpga_adc_telemetry.phys.v_wind_mps,       &sADC),
			_FLOAT_R_("Turret Cosine (cos_fi)",    g_fpga_adc_telemetry.phys.cos_fi,           &sADC),
			_BOOL_R_ ("ADC Data valid",            g_fpga_adc_telemetry.is_valid,              &sADC),

		/* --- BMP280 BAROMETRIC SENSOR SUBTREE --- */
		_STRING_R_ ("Sensor BMP280", sBMP, &sDEV),
			_FLOAT_R_("BMP280 Temperature [°C]",   g_bmp280_telemetry.temperature_c, &sBMP),
			_FLOAT_R_("BMP280 Pressure [Pa]",      g_bmp280_telemetry.pressure_pa,   &sBMP),
			_FLOAT_R_("BMP280 Pressure [mmHg]",    g_bmp280_telemetry.pressure_mmhg, &sBMP),
			_FLOAT_R_("BMP280 Altitude [m]",       g_bmp280_telemetry.altitude_m,    &sBMP),
			_BOOL_R_ ("BMP280 data valid",         g_bmp280_telemetry.is_valid,      &sBMP),
			_BOOL_R_ ("BMP280 sensor present",     g_bmp280_present,                 &sBMP),

		/* --- DIAGNOSTICS & SYSTEM TELEMETRY SUBTREE --- */
		_STRING_R_ ("Diagnostics", sDEBUG, &sDEV),
			_FLOAT_R_("CPU temperature [°C]",      cpu_temperature,        &sDEBUG),
			_FLOAT_R_("ADC voltage [V]",           adc_voltage,            &sDEBUG),
			_U32_RW_ ("PSPM 400Hz task counter",   pspm_task_counter,      &sDEBUG),
			_U32_R_  ("Host TX packets",           host_stats.tx_count,    &sDEBUG),
			_U32_R_  ("Host RX packets",           host_stats.rx_count,    &sDEBUG),
			_U32_R_  ("Host RX CRC errors",        host_stats.rx_crc_err,  &sDEBUG),
			_U32_R_  ("Host RX XOR errors",        host_stats.rx_xor_err,  &sDEBUG),
			_U32_R_  ("Host RX dropped bytes",     host_stats.rx_drop,     &sDEBUG),
			_U16_R_  ("FPGA status word",          g_fpga_status_word,     &sDEBUG),
			_U32_R_  ("ST evt ADC watchdog",       g_fpga_st_evt[IC_ST_EVT_ADC_ERR], &sDEBUG),
			_U32_R_  ("ST evt RES watchdog",       g_fpga_st_evt[IC_ST_EVT_RES_ERR], &sDEBUG),
			_U32_R_  ("ST evt RES overrun",        g_fpga_st_evt[IC_ST_EVT_RES_OVR], &sDEBUG),
			_U32_R_  ("ST evt COMP 1-2",           g_fpga_st_evt[IC_ST_EVT_ERR_12],  &sDEBUG),
			_U32_R_  ("ST evt COMP 2-3",           g_fpga_st_evt[IC_ST_EVT_ERR_23],  &sDEBUG),

		/* --- DAC DEBUG WAVEFORM GENERATOR SUBTREE (1 ms task, see app_tasks.c) --- */
		_STRING_R_ ("DAC debug gen", sDBG, &sDEV),
			_BOOL_RW_  ("DBG Mode (on/off)",       g_app_dbg_gen.mode,           &sDBG),
			_BYTE_RW_  ("DBG Form (0..6)",         g_app_dbg_gen.gen.waveform,   &sDBG),
			_FLOAT_RW_ ("DBG Freq [Hz 0.01..30]",  g_app_dbg_gen.gen.freq_hz,    &sDBG),
			_FLOAT_RW_ ("DBG Amplitude [0..32768]", g_app_dbg_gen.gen.amplitude, &sDBG),
			_FLOAT_RW_ ("DBG Bias [-32768..32768]", g_app_dbg_gen.gen.offset,    &sDBG),
			_BYTE_RW_  ("DBG DAC ch [1..10]",      g_app_dbg_gen.dac_ch,         &sDBG),
			_FLOAT_R_  ("DBG Last value [code]",   g_app_dbg_gen.gen.value,      &sDBG),
SIGNALS_END(DSPA_SIGNALS_NAME)

/* ========================================================================= */
/*  PUBLIC API IMPLEMENTATION                                                */
/* ========================================================================= */

int init_terminal_signals(void) {
	return SIG_INIT(DSPA_SIGNALS_NAME);
}

void signal_change_handler(void *s) {
	// RDC config parameter write guard: validate every edit against
	// RDC_ValidateConfig ranges and snap back the value if it is out of bounds.
	char *base  = (char *)&g_rdc_math_cfg;
	char *write = (char *)s;
	if (write >= base && write < base + (ptrdiff_t)sizeof(rdc_math_cfg_t)) {
		static rdc_math_cfg_t last_good = {0};
		static bool           seeded    = false;
		if (!seeded) {
			// Baseline = validated live config loaded at boot (post RDC_Init state)
			last_good = g_rdc_math_cfg;
			seeded    = true;
		}
		if (RDC_ValidateConfig(&g_rdc_math_cfg)) {
			last_good = g_rdc_math_cfg; // Accept: remember as rollback baseline
		} else {
			g_rdc_math_cfg = last_good; // Reject: restore previous valid values
		}
	}

	// RDC PLL Configuration Apply Controller (validate -> flash -> recalc Kp/Ki)
	if (s == &eAssist_rdc_apply_cmd) {
		if (eAssist_rdc_apply_cmd == 1) {
			// save_setting() validates via RDC_ValidateConfig before any flash write
			if (!save_setting()) {
				// Bad parameters: roll back live config to last known good image
				g_rdc_math_cfg = dev_cfg.item.rdc_cfg;
			}
		}
		eAssist_rdc_apply_cmd = 0; // Return command to NoOp
	}

	// Debug generator write guard: validate every field edit against the
	// safe bench ranges and snap the whole config back on violation.
	char *dbase  = (char *)&g_app_dbg_gen;
	char *dwrite = (char *)s;
	if (dwrite >= dbase && dwrite < dbase + (ptrdiff_t)sizeof(app_dbg_gen_cfg_t)) {
		/* Shadow starts from known-valid defaults, NOT from the first write:
		   an invalid very first value would otherwise become the baseline.
		   Must match g_app_dbg_gen static initializer in app_tasks.c */
		static app_dbg_gen_cfg_t dbg_shadow = {
		    .mode   = false,
		    .dac_ch = 1U,
		    .gen    = { 1000.0f, 0.0f, 10.0f, T_W_NONE, 0.0f, 0U, 0.0f },
		};

		ptrdiff_t off = dwrite - dbase;
		bool ok = true; /* mode is bool: every legal write to it is valid */

		if (off == (ptrdiff_t)offsetof(app_dbg_gen_cfg_t, dac_ch)) {
			ok = (g_app_dbg_gen.dac_ch >= 1U && g_app_dbg_gen.dac_ch <= 10U);
		} else if (off == (ptrdiff_t)offsetof(app_dbg_gen_cfg_t, gen) +
		                   (ptrdiff_t)offsetof(CTestGenerator, waveform)) {
			ok = (g_app_dbg_gen.gen.waveform <= T_W_NOISE);
		} else if (off == (ptrdiff_t)offsetof(app_dbg_gen_cfg_t, gen) +
		                   (ptrdiff_t)offsetof(CTestGenerator, freq_hz)) {
			ok = (g_app_dbg_gen.gen.freq_hz >= 0.01f && g_app_dbg_gen.gen.freq_hz <= 30.0f);
		} else if (off == (ptrdiff_t)offsetof(app_dbg_gen_cfg_t, gen) +
		                   (ptrdiff_t)offsetof(CTestGenerator, amplitude)) {
			ok = (g_app_dbg_gen.gen.amplitude >= 0.0f && g_app_dbg_gen.gen.amplitude <= 32768.0f);
		} else if (off == (ptrdiff_t)offsetof(app_dbg_gen_cfg_t, gen) +
		                   (ptrdiff_t)offsetof(CTestGenerator, offset)) {
			ok = (g_app_dbg_gen.gen.offset >= -32768.0f && g_app_dbg_gen.gen.offset <= 32768.0f);
		}

		if (ok) {
			dbg_shadow = g_app_dbg_gen; // Accept: remember current as baseline
		} else {
			g_app_dbg_gen = dbg_shadow; // Reject: restore last accepted values
		}
	}
}
