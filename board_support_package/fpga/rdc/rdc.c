/**
 ******************************************************************************
 * @file    rdc.c
 * @brief   Type-II Tracking Phase-Locked Loop (PLL) Resolver DSP Implementation.
 *          100% mathematically equivalent to verified Zynq implementation.
 *          Target MCU: STM32F411 (Cortex-M4 @ 100 MHz, FreeRTOS).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "rdc.h"
#include <math.h>
#include <string.h>

#define RDC_AXIS_COUNT 2
#define AXIS_ROLL      0
#define AXIS_PITCH     1

/**
 * @brief State Machine for one axis Type-II Tracking Observer
 */
typedef struct {
    float kp;                 /* Proportional gain: Kp = 2 * zeta * omega_n */
    float ki;                 /* Integral gain: Ki = omega_n^2 */
    float dt;                 /* Calculation step: dt = 1 / CarrierFreq */
    float max_velocity;       /* Velocity clamp limit [rad/s] */

    float est_angle;          /* Current estimated angle [rad] */
    float est_velocity;       /* Current estimated velocity [rad/s] */
    float integrator;         /* Anti-windup integrator state */
    float track_error;        /* Phase detector tracking error */
} rdc_tracker_t;

/* Global configurations and telemetry storage */
rdc_math_cfg_t  g_rdc_math_cfg  = {0};
rdc_telemetry_t g_rdc_telemetry = {0};

/* Private tracking observers for Roll and Pitch */
static rdc_tracker_t trackers[RDC_AXIS_COUNT];

/* ========================================================================= */
/*  PRIVATE DSP OBSERVER CORE (100% Exact Zynq Algorithm)                    */
/* ========================================================================= */

/**
 * @brief Executes one iteration of Type-II PLL Tracking Loop for a single axis.
 */
static void rdc_tracker_update(rdc_tracker_t *tracker, float sin_raw, float cos_raw) {
    // 1. Amplitude Normalization
    float amplitude = sqrtf(sin_raw * sin_raw + cos_raw * cos_raw);

    // Broken cable / loss of carrier detector
    if (amplitude < RDC_MIN_AMPLITUDE_THRESHOLD) {
        tracker->est_velocity = 0.0f;
        return;
    }

    float sin_norm = sin_raw / amplitude;
    float cos_norm = cos_raw / amplitude;

    // 2. Mathematical angle estimate estimation
    float sin_est = sinf(tracker->est_angle);
    float cos_est = cosf(tracker->est_angle);

    // 3. Phase Detector Error: e = sin(theta)*cos(est) - cos(theta)*sin(est) = sin(theta - est)
    float error = (sin_norm * cos_est) - (cos_norm * sin_est);
    tracker->track_error = error;

    // 4. PI Tracking Controller with Anti-Windup Clamping
    tracker->integrator += error * tracker->ki * tracker->dt;
    tracker->integrator = CLAMP(tracker->integrator, -tracker->max_velocity, tracker->max_velocity);

    float velocity_next = (error * tracker->kp) + tracker->integrator;
    tracker->est_velocity = CLAMP(velocity_next, -tracker->max_velocity, tracker->max_velocity);

    // 5. Position Integrator: theta = theta + omega * dt
    tracker->est_angle += tracker->est_velocity * tracker->dt;

    // 6. Angle Normalization to [-PI, +PI]
    while (tracker->est_angle > _PI)  tracker->est_angle -= _2PI;
    while (tracker->est_angle < -_PI) tracker->est_angle += _2PI;
}

/* ========================================================================= */
/*  PUBLIC API IMPLEMENTATION                                                */
/* ========================================================================= */

/* Range gate for every PLL parameter: carrier 375..425 Hz, bandwidth > 0.1 Hz
   and <= carrier/2 (Nyquist), damping 0.1..5.0, speed limit 0..1000 deg/s.
   Called by terminal write-guard, save_setting() and RDC_Init(). */
bool RDC_ValidateConfig(const rdc_math_cfg_t *cfg) {
    if (cfg == NULL) return false;

    if (cfg->carrier_freq_hz < RDC_MIN_CARRIER_FREQ_HZ || cfg->carrier_freq_hz > RDC_MAX_CARRIER_FREQ_HZ) {
        return false;
    }
    if (cfg->loop_bandwidth_hz <= RDC_MIN_LOOP_BW_HZ || cfg->loop_bandwidth_hz > (cfg->carrier_freq_hz / 2.0f)) {
        return false;
    }
    if (cfg->damping_ratio < RDC_MIN_DAMPING_RATIO || cfg->damping_ratio > RDC_MAX_DAMPING_RATIO) {
        return false;
    }
    if (cfg->max_speed_dps <= RDC_MIN_SPEED_DPS || cfg->max_speed_dps > RDC_MAX_SPEED_DPS) {
        return false;
    }
    return true;
}

/* Configure both tracking axes: accepts a validated cfg or falls back to
   factory defaults, then computes the gains Kp = 2*zeta*wn, Ki = wn^2,
   dt = 1/F_carrier and zeroes observer states and telemetry output. */
void RDC_Init(const rdc_math_cfg_t *cfg) {
    if (cfg != NULL && RDC_ValidateConfig(cfg)) {
        g_rdc_math_cfg = *cfg;
    } else {
        // Load default parameters
        g_rdc_math_cfg.carrier_freq_hz   = RDC_DEFAULT_CARRIER_FREQ_HZ;
        g_rdc_math_cfg.loop_bandwidth_hz = RDC_DEFAULT_LOOP_BANDWIDTH_HZ;
        g_rdc_math_cfg.damping_ratio     = RDC_DEFAULT_DAMPING_RATIO;
        g_rdc_math_cfg.max_speed_dps     = RDC_DEFAULT_MAX_SPEED_DPS;
        g_rdc_math_cfg.add_PI            = RDC_DEFAULT_ADD_PI;
    }

    // Calculate Type-II PLL coefficients
    float omega_n      = _2PI * g_rdc_math_cfg.loop_bandwidth_hz;
    float kp_calc      = 2.0f * g_rdc_math_cfg.damping_ratio * omega_n;
    float ki_calc      = omega_n * omega_n;
    float dt_calc      = 1.0f / g_rdc_math_cfg.carrier_freq_hz;
    float max_vel_rads = DEG_TO_RAD(g_rdc_math_cfg.max_speed_dps);

    // Apply coefficients to both tracking axes
    for (int i = 0; i < RDC_AXIS_COUNT; i++) {
        trackers[i].kp           = kp_calc;
        trackers[i].ki           = ki_calc;
        trackers[i].dt           = dt_calc;
        trackers[i].max_velocity = max_vel_rads;

        trackers[i].est_angle    = 0.0f;
        trackers[i].est_velocity = 0.0f;
        trackers[i].integrator   = 0.0f;
        trackers[i].track_error  = 0.0f;
    }

    memset(&g_rdc_telemetry, 0, sizeof(rdc_telemetry_t));
}

/* 400 Hz real-time step (run per FPGA data-ready IRQ): stores raw integrals,
   then checks the carrier via the period sample-count window (200..600 Hz at
   100 kSPS). Out-of-window freezes outputs and resets observers; otherwise
   runs both PLL axes and publishes angles, speeds and BAM16 words. */
void RDC_Process(const FpgaDmaTelemetryPacket_t *raw_dma) {
    if (raw_dma == NULL) return;

    // 1. Copy raw hardware integrals
    g_rdc_telemetry.sin_roll    = raw_dma->res_sin_roll;
    g_rdc_telemetry.cos_roll    = raw_dma->res_cos_roll;
    g_rdc_telemetry.sin_pitch   = raw_dma->res_sin_pitch;
    g_rdc_telemetry.cos_pitch   = raw_dma->res_cos_pitch;
    g_rdc_telemetry.samples_cnt = raw_dma->res_samples_cnt;

    // 2. Carrier Frequency Watchdog Window (200 Hz .. 600 Hz)
    if (raw_dma->res_samples_cnt < RDC_MIN_SAMPLES_LIMIT ||
        raw_dma->res_samples_cnt > RDC_MAX_SAMPLES_LIMIT)
    {
        // Frequency out of specification: reset outputs and freeze
        g_rdc_telemetry.is_valid        = false;
        g_rdc_telemetry.roll_angle_rad  = 0.0f;
        g_rdc_telemetry.roll_angle_deg  = 0.0f;
        g_rdc_telemetry.roll_speed_dps  = 0.0f;
        g_rdc_telemetry.roll_error      = 0.0f;
        g_rdc_telemetry.roll_angle_bf   = BAM16_180_DEG;

        g_rdc_telemetry.pitch_angle_rad = 0.0f;
        g_rdc_telemetry.pitch_angle_deg = 0.0f;
        g_rdc_telemetry.pitch_speed_dps = 0.0f;
        g_rdc_telemetry.pitch_error     = 0.0f;
        g_rdc_telemetry.pitch_angle_bf  = BAM16_180_DEG;

        for (int i = 0; i < RDC_AXIS_COUNT; i++) {
            trackers[i].est_angle    = 0.0f;
            trackers[i].est_velocity = 0.0f;
            trackers[i].integrator   = 0.0f;
            trackers[i].track_error  = 0.0f;
        }
        return;
    }

    g_rdc_telemetry.is_valid = true;

    // 3. Process Roll Axis (AXIS_ROLL)
    rdc_tracker_update(&trackers[AXIS_ROLL], (float)raw_dma->res_sin_roll, (float)raw_dma->res_cos_roll);
    g_rdc_telemetry.roll_angle_rad = trackers[AXIS_ROLL].est_angle;
    g_rdc_telemetry.roll_angle_deg = RAD_TO_DEG(trackers[AXIS_ROLL].est_angle);
    g_rdc_telemetry.roll_speed_dps = RAD_TO_DEG(trackers[AXIS_ROLL].est_velocity);
    g_rdc_telemetry.roll_error     = trackers[AXIS_ROLL].track_error;

    if (g_rdc_math_cfg.add_PI) {
        g_rdc_telemetry.roll_angle_bf = RAD_TO_BAM16(trackers[AXIS_ROLL].est_angle) + BAM16_180_DEG;
    } else {
        g_rdc_telemetry.roll_angle_bf = RAD_TO_BAM16(trackers[AXIS_ROLL].est_angle);
    }

    // 4. Process Pitch Axis (AXIS_PITCH)
    rdc_tracker_update(&trackers[AXIS_PITCH], (float)raw_dma->res_sin_pitch, (float)raw_dma->res_cos_pitch);
    g_rdc_telemetry.pitch_angle_rad = trackers[AXIS_PITCH].est_angle;
    g_rdc_telemetry.pitch_angle_deg = RAD_TO_DEG(trackers[AXIS_PITCH].est_angle);
    g_rdc_telemetry.pitch_speed_dps = RAD_TO_DEG(trackers[AXIS_PITCH].est_velocity);
    g_rdc_telemetry.pitch_error     = trackers[AXIS_PITCH].track_error;

    if (g_rdc_math_cfg.add_PI) {
        g_rdc_telemetry.pitch_angle_bf = RAD_TO_BAM16(trackers[AXIS_PITCH].est_angle) + BAM16_180_DEG;
    } else {
        g_rdc_telemetry.pitch_angle_bf = RAD_TO_BAM16(trackers[AXIS_PITCH].est_angle);
    }
}

/* Program the AD7367 input routing matrix (8-bit selector) of the resolver
   sampling frontend in FPGA. */
FPGA_Status_t RDC_SetRouting(FPGA_HandleTypeDef *hbridge, uint8_t sel_mask) {
    if (!hbridge) return FPGA_ERROR;
    // Write routing mask to Device ID 4, Reg 0x00 (Bits [7:0])
    return FPGA_Write_Poll(hbridge, ADDR_S_RES_CTRL, (uint16_t)sel_mask, 100);
}

/* Read-only view of the latest processed resolver telemetry. */
const rdc_telemetry_t* RDC_GetTelemetry(void) {
    return &g_rdc_telemetry;
}
