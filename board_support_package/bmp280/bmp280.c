/**
 ******************************************************************************
 * @file    bmp280.c
 * @brief   Bosch BMP280 Driver Implementation with Internal Encapsulation.
 *          Target: SPI Master Mode (SPI3 @ 6.25 MHz, 8-bit, Mode 00).
 *          Dedicated SPI3 bus: SCK=PB12(AF7), MISO=PC11(AF6), MOSI=PC12(AF6),
 *          CS=PC10 (GPIO). Shares no resources with the FPGA SPI1 bus.
 *          Datasheet reference: BST-BMP280-DS001-10 (rev 1.13).
 *          All comments in English.
 ******************************************************************************
 */

#include "bmp280.h"
#include <math.h>
#include <string.h>

/* Private Driver Context */
static SPI_HandleTypeDef      *g_bmp_spi   = NULL;  /* Dedicated SPI3 handle          */
static const Pin_Descriptor_t *g_bmp_cs    = NULL;  /* Chip select descriptor (PC10)  */
static BMP280_Calib_t          g_calib;             /* Factory NVM calibration set    */
static int32_t                 g_t_fine    = 0;     /* Fine-temp intermediate shared  */
                                                    /* by pressure compensation (7.3) */
static bool                    g_connected = false; /* Init sequence completed flag   */

/* Global telemetry snapshot: BC packet builder + terminal signals */
BMP280_Data_t              g_bmp280_telemetry = {0};

/* Chip presence: set true once a valid chip ID was read in BMP280_Init */
bool                       g_bmp280_present   = false;

/* ID-watchdog bookkeeping (Process runs at 10 Hz in background task) */
#define BMP280_HEALTH_DIV  10U           /* 10 calls @10 Hz = 1 s watchdog   */
static uint8_t             s_health_div  = 0;
static uint8_t             s_retry_div   = 0;      /* auto-reinit cadence    */

/* Physical plausibility gates: a broken/flapping MISO still yields HAL_OK
   bursts, and the Bosch formula happily turns 0xFF garbage into absurd
   pressures (e.g. 1200 mmHg). Out-of-range frames are never published.
   300..1100 hPa is the datasheet operating range (Key Parameters);
   -40..+85 is the spec operating range of the part. */
#define BMP280_PRESS_MIN_PA  30000.0f
#define BMP280_PRESS_MAX_PA  110000.0f
#define BMP280_TEMP_MIN_C    (-40.0f)
#define BMP280_TEMP_MAX_C    85.0f

/* ========================================================================= */
/*  LOW-LEVEL SPI HELPERS                                                    */
/*  SPI frame (4-wire, datasheet section 3.2):                               */
/*    byte0 = R/W bit [7] + register address [6:0], MSB first                */
/*    R/W = 0 -> write, R/W = 1 -> read; CS held low for the whole frame     */
/* ========================================================================= */

/* CS active-low: open an SPI frame */
static void _cs_low(void)  { if (g_bmp_cs) PIN_Reset_F(g_bmp_cs); }

/* CS inactive-high: close frame + latch shadow registers (sec 3.9) */
static void _cs_high(void) { if (g_bmp_cs) PIN_Set_F(g_bmp_cs); }

/* Write one register: [addr&0x7F][value], one CS frame, 50 ms timeout */
static HAL_StatusTypeDef _write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = { reg & 0x7FU, value }; // Bit 7 = 0 for SPI Write
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(g_bmp_spi, tx_buf, 2, 50);
    _cs_high();
    return status;
}

/* Read one register: [addr|0x80][dummy], answer arrives with the dummy byte */
static HAL_StatusTypeDef _read_reg(uint8_t reg, uint8_t *val) {
    uint8_t tx_buf[2] = { reg | 0x80U, 0x00U }; // Bit 7 = 1 for SPI Read
    uint8_t rx_buf[2] = { 0 };
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(g_bmp_spi, tx_buf, rx_buf, 2, 50);
    _cs_high();
    if (status == HAL_OK && val) *val = rx_buf[1];
    return status;
}

/* Burst-read N consecutive registers in ONE CS frame.
   Required for 0xF7..0xFC: a single frame prevents byte mix-up when a new
   measurement completes mid-readout (shadow registers, datasheet sec 3.9). */
static HAL_StatusTypeDef _read_burst(uint8_t start_reg, uint8_t *buffer, uint16_t len) {
    uint8_t tx_addr = start_reg | 0x80U; // Bit 7 = 1 for SPI Read
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(g_bmp_spi, &tx_addr, 1, 50);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(g_bmp_spi, buffer, len, 50);
    }
    _cs_high();
    return status;
}

/* ========================================================================= */
/*  ID WATCHDOG                                                              */
/*  A dead/unpowered slave still returns HAL_OK on SPI (master drives the    */
/*  clock), so read errors alone cannot prove loss - the chip ID is the only */
/*  cheap liveness proof. Runs once per second from BMP280_Process().        */
/* ========================================================================= */

/* Periodic ID (0xD0) check, once per second: good ID -> present restored;
   bad/failed ID -> present & is_valid cleared so garbage never reaches the
   BC packet or terminal as valid data. */
static void _health_check(void) {
    uint8_t id = 0;

    if (++s_health_div < BMP280_HEALTH_DIV) return;
    s_health_div = 0;

    if ((_read_reg(BMP280_REG_ID, &id) == HAL_OK) &&
        (id == BMP280_CHIP_ID_VAL || id == BME280_CHIP_ID_VAL)) {
        g_bmp280_present = true;
    } else {
        g_bmp280_present            = false;
        g_bmp280_telemetry.is_valid = false;
        /* Drop stale/garbage numbers so the terminal shows 0 + invalid,
           not a frozen absurd value. Force full re-init on recovery: a
           brown-out or CS flap can leave ctrl/config at reset defaults. */
        g_bmp280_telemetry.temperature_c = 0.0f;
        g_bmp280_telemetry.pressure_pa   = 0.0f;
        g_bmp280_telemetry.pressure_mmhg = 0.0f;
        g_bmp280_telemetry.altitude_m    = 0.0f;
        g_connected = false;
    }
}

/* ========================================================================= */
/*  BOSCH INTEGER COMPENSATION (Datasheet sections 7.2 / 7.3)                */
/*  Temperature: 32-bit fixed point, result in 0.01 degC units.              */
/*  Pressure:    64-bit fixed point (highest accuracy on 32-bit MCU),        */
/*               result in Q24.8 Pascal format.                              */
/*  NOTE: pressure compensation consumes g_t_fine produced by the            */
/*  temperature call, therefore temperature MUST be compensated first.       */
/* ========================================================================= */

/* Bosch 32-bit formula: raw 20-bit ADC -> degC x 100 (2512 = 25.12 degC).
   Side effect: stores g_t_fine used by the pressure formula -> MUST run
   before _compensate_pressure() in the same cycle. */
static int32_t _compensate_temperature(int32_t adc_T) {
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)g_calib.dig_T1 << 1))) * ((int32_t)g_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)g_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)g_calib.dig_T1))) >> 12) *
            ((int32_t)g_calib.dig_T3)) >> 14;
    g_t_fine = var1 + var2;
    return (g_t_fine * 5 + 128) >> 8;
}

/* Bosch 64-bit formula (highest accuracy on Cortex-M4F): raw 20-bit ADC ->
   pressure in Q24.8 fixed point (divide by 256 for Pa). Reads g_t_fine set by
   _compensate_temperature(), so temperature must be compensated first.
   Returns 0 when dig_P1 is 0 to guard the division. */
static uint32_t _compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)g_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)g_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)g_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)g_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)g_calib.dig_P3) >> 8) + ((var1 * (int64_t)g_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)g_calib.dig_P1) >> 33;
    if (var1 == 0) return 0; // Avoid divide by zero
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)g_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)g_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)g_calib.dig_P7) << 4);
    return (uint32_t)p;
}

/* ========================================================================= */
/*  PUBLIC API IMPLEMENTATION                                                */
/* ========================================================================= */

/* One-time bring-up: ID check -> soft reset -> load 24B calibration -> config
   -> NORMAL mode. Returns BMP280_ID_ERROR if no valid chip answers (absent
   sensor), BMP280_ERROR on NULL arg or any SPI failure.
   Blocking (~30 ms via PIN_Delay): call from board init only, never from the
   400 Hz loop. Config 0xF5 is written while still in SLEEP mode, because a
   0xF5 write in NORMAL mode may be ignored (datasheet sec 4.3.5). */
BMP280_Status_t BMP280_Init(SPI_HandleTypeDef *hspi, const Pin_Descriptor_t *cs_pin) {
    uint8_t chip_id = 0;
    uint8_t calib_raw[24];

    if (!hspi || !cs_pin) return BMP280_ERROR;

    g_bmp_spi   = hspi;
    g_bmp_cs    = cs_pin;
    g_connected = false;
    g_bmp280_telemetry.is_valid = false;

    _cs_high();
    PIN_Delay(10);

    // 1. Read & verify Chip ID (0x58 for BMP280, 0x60 also accepted for BME280)
    g_bmp280_present = false;
    if (_read_reg(BMP280_REG_ID, &chip_id) != HAL_OK) return BMP280_ERROR;
    if (chip_id != BMP280_CHIP_ID_VAL && chip_id != BME280_CHIP_ID_VAL) return BMP280_ID_ERROR;
    g_bmp280_present = true;

    // 2. Power-on software reset
    _write_reg(BMP280_REG_RESET, BMP280_CMD_SOFT_RESET);
    PIN_Delay(20);

    // 3. Read 24 calibration bytes (Table 17), little-endian LSB/MSB pairs
    if (_read_burst(BMP280_REG_CALIB00, calib_raw, 24) != HAL_OK) return BMP280_ERROR;

    g_calib.dig_T1 = (uint16_t)(calib_raw[0]  | (calib_raw[1]  << 8));
    g_calib.dig_T2 = (int16_t) (calib_raw[2]  | (calib_raw[3]  << 8));
    g_calib.dig_T3 = (int16_t) (calib_raw[4]  | (calib_raw[5]  << 8));
    g_calib.dig_P1 = (uint16_t)(calib_raw[6]  | (calib_raw[7]  << 8));
    g_calib.dig_P2 = (int16_t) (calib_raw[8]  | (calib_raw[9]  << 8));
    g_calib.dig_P3 = (int16_t) (calib_raw[10] | (calib_raw[11] << 8));
    g_calib.dig_P4 = (int16_t) (calib_raw[12] | (calib_raw[13] << 8));
    g_calib.dig_P5 = (int16_t) (calib_raw[14] | (calib_raw[15] << 8));
    g_calib.dig_P6 = (int16_t) (calib_raw[16] | (calib_raw[17] << 8));
    g_calib.dig_P7 = (int16_t) (calib_raw[18] | (calib_raw[19] << 8));
    g_calib.dig_P8 = (int16_t) (calib_raw[20] | (calib_raw[21] << 8));
    g_calib.dig_P9 = (int16_t) (calib_raw[22] | (calib_raw[23] << 8));

    // 4. Config (t_sb=62.5 ms, IIR=x16). Must be written in SLEEP mode:
    //    writes to 0xF5 in NORMAL mode may be ignored (datasheet section 4.3.5).
    if (_write_reg(BMP280_REG_CONFIG, 0x30U) != HAL_OK) return BMP280_ERROR;

    // 5. Enter NORMAL mode (Temp x2, Press x16): sensor self-cycles at ~10 Hz
    if (_write_reg(BMP280_REG_CTRL_MEAS, 0x57U) != HAL_OK) return BMP280_ERROR;

    g_connected = true;
    return BMP280_OK;
}

/* Periodic read at ~10 Hz: burst 6 data regs, compensate, publish to
   g_bmp280_telemetry. Returns BMP280_ERROR if not connected or SPI fails; on
   SPI failure is_valid is cleared so consumers see the loss, not stale data. */
BMP280_Status_t BMP280_Process(void) {
    uint8_t raw_data[6];
    int32_t raw_press, raw_temp;

    if (!g_connected) {
        /* Auto-recovery: retry full bring-up once per second (handles
           hot-plug and re-powered sensor without an MCU reset). The 30 ms
           blocking inside BMP280_Init is acceptable at this rate: the caller
           is the 10 Hz background task, never the 400 Hz control loop. */
        if (++s_retry_div >= BMP280_HEALTH_DIV) {
            s_retry_div = 0;
            if (g_bmp_spi != NULL) {
                (void)BMP280_Init(g_bmp_spi, g_bmp_cs);
            }
        }
        return BMP280_ERROR;
    }

    // Burst read 6 data registers starting at 0xF7 (Press MSB..Temp XLSB)
    if (_read_burst(BMP280_REG_PRESS_MSB, raw_data, 6) != HAL_OK) {
        g_bmp280_telemetry.is_valid = false;
        _health_check();    /* keep presence info fresh even during read loss */
        return BMP280_ERROR;
    }

    // Assemble 20-bit raw ADC values (MSB<<12 | LSB<<4 | XLSB>>4), positive
    raw_press = (int32_t)(((uint32_t)raw_data[0] << 12) | ((uint32_t)raw_data[1] << 4) | ((uint32_t)raw_data[2] >> 4));
    raw_temp  = (int32_t)(((uint32_t)raw_data[3] << 12) | ((uint32_t)raw_data[4] << 4) | ((uint32_t)raw_data[5] >> 4));

    // Temperature first: it produces g_t_fine that the pressure formula needs
    int32_t comp_temp = _compensate_temperature(raw_temp);
    uint32_t comp_press_q24_8 = _compensate_pressure(raw_press);

    // Publish physical units: degC, Pa and the mmHg unit used by the BC packet
    float temp_c   = (float)comp_temp / 100.0f;
    float press_pa = (float)comp_press_q24_8 / 256.0f;

    /* Plausibility gate: reject frames outside the physical range (broken
       MISO gives HAL_OK + garbage). Previous numbers stay displayed but
       the valid flag goes false. */
    if (press_pa < BMP280_PRESS_MIN_PA || press_pa > BMP280_PRESS_MAX_PA ||
        temp_c   < BMP280_TEMP_MIN_C   || temp_c   > BMP280_TEMP_MAX_C) {
        g_bmp280_telemetry.is_valid = false;
        _health_check();
        return BMP280_ERROR;
    }

    g_bmp280_telemetry.temperature_c = temp_c;
    g_bmp280_telemetry.pressure_pa   = press_pa;
    g_bmp280_telemetry.pressure_mmhg = press_pa * 0.00750062f;

    // Barometric altitude vs ISA sea-level reference (P is > 0 after the gate)
    g_bmp280_telemetry.altitude_m = 44330.0f * (1.0f - powf((press_pa / BMP280_SEA_LEVEL_PRESSURE_PA), 0.19029495f));

    g_bmp280_telemetry.is_valid = true;

    /* Watchdog runs last: a bad ID overrides the just-set is_valid, because
       HAL-OK bursts from an unpowered chip return garbage that must not pass
       as valid data. */
    _health_check();
    return BMP280_OK;
}

/* true once BMP280_Init completed; means bring-up OK, NOT that data are fresh */
bool BMP280_IsConnected(void) {
    return g_connected;
}

/* Validity-gated snapshot, preferred API for the BC packet builder: copies
   g_bmp280_telemetry only when valid, else false with *out_data untouched. */
bool BMP280_GetTelemetry(BMP280_Data_t *out_data) {
    if (!out_data || !g_bmp280_telemetry.is_valid) return false;
    *out_data = g_bmp280_telemetry;
    return true;
}

/* Cached value, NOT validity-gated: may be stale, diagnostics/terminal only */
float BMP280_GetTemperature(void) {
    return g_bmp280_telemetry.temperature_c;
}

/* Cached pressure (Pa), may be stale; use GetTelemetry() for protocol use */
float BMP280_GetPressure(void) {
    return g_bmp280_telemetry.pressure_pa;
}

/* Cached altitude (m), may be stale; use GetTelemetry() for protocol use */
float BMP280_GetAltitude(void) {
    return g_bmp280_telemetry.altitude_m;
}
