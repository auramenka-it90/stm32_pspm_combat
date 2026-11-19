/**
 ******************************************************************************
 * @file    app_tasks.c
 * @brief   PSPM application task bodies. Kept outside the CubeMX-generated
 *          freertos.c: the task entries there stay one-line calls, so
 *          regeneration of freertos.c cannot remove application logic.
 *          - app_default_task     : POST + fatal blink + 400 Hz FPGA pipeline
 *          - app_terminal_task    : DSPA terminal engine (1 ms period inside)
 *          - app_temperature_task : BMP280 + die temperature at 10 Hz
 *          - app_freq_generator_task : 1 ms debug waveform generator task
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "app_tasks.h"

#include "board_support_package.h"
#include "pin_mgmt.h"
#include "stm32_2_fpga_spi_bridge.h"
#include "fpga_control.h"
#include "terminal.h"
#include "ic.h"
#include "rdc.h"
#include "fpga_adc.h"
#include "host.h"
#include "bmp280.h"

/* Global 400 Hz control loop execution counter */
uint32_t pspm_task_counter = 0;

/* Debug waveform generator control (terminal sDBG subtree edits it live).
   Defaults are in-range valid values and must match the guard shadow copy. */
app_dbg_gen_cfg_t g_app_dbg_gen = {
    .mode   = false,
    .dac_ch = 1U,
    .gen    = { 1000.0f, 0.0f, 10.0f, T_W_NONE, 0.0f, 0U, 0.0f },
};

/* Global FPGA Bridge handle from board_support_package.c */
extern FPGA_HandleTypeDef hfpga_bridge;

/* Single-LED heartbeat indication on FPGA LEDs (VD35 green / VD36 yellow / VD37 red):
   G 1 Hz = loop alive, carrier present, all ok
   Y 1 Hz = no 400 Hz carrier (generator off / UON lost)
   R 1 Hz = BC UART link silent
   R 5 Hz = fatal SPI/FPGA runtime error
   Short 100 ms pulses; a frozen (non-pulsing) LED means dead code, not a mode.
   SPI write happens only on mask change. */
static void led_indication_update(void)
{
    static uint8_t last_mask = 0xFFU; /* bit0=R, bit1=Y, bit2=G; 0xFF forces first write */
    const uint32_t now = HAL_GetTick();
    uint8_t mask;
    uint32_t period_ms;
    bool pulse;

    if (IC_GetSystemStatusByte() & IC_STATUS_FPGA_ERROR) {
        mask = 0x1U; period_ms = 200U;   /* fatal: 5 Hz */
    } else if (is_bc_link_error) {
        mask = 0x1U; period_ms = 1000U;  /* BC link lost: 1 Hz */
    } else if (!IC_Is400HzValid()) {
        mask = 0x2U; period_ms = 1000U;  /* no phase: 1 Hz */
    } else {
        mask = 0x4U; period_ms = 1000U;  /* all ok: 1 Hz */
    }

    pulse = ((now % period_ms) < 100U);
    mask  = pulse ? mask : 0U;

    if (mask != last_mask) {
        last_mask = mask;
        (void)FPGA_Debug_Write_LEDs(&hfpga_bridge, (mask & 0x1U) != 0U,
                                    (mask & 0x2U) != 0U,
                                    (mask & 0x4U) != 0U, 10);
    }
}

/* Master 400 Hz real-time control sequencer (T = 2.500 ms).
   Runs after CubeMX startup; POST failure ends in a fatal blink loop. */
void app_default_task(void)
{
    uint32_t board_faults = init_hardware();

    /* Non-fatal faults (BMP280) are logged only and MUST NOT stop the loop */
    if ((board_faults & ~_B_NONFATAL_FAULTS_) != 0U) {
        /* POST failed: IC status machinery cannot be trusted with dead hardware.
           Direct blink: red FPGA LED 5 Hz + MCU green heartbeat 1 Hz. If SPI/FPGA
           is dead the FPGA LEDs stay dark; the MCU LED proves the code is alive. */
        uint32_t beat = 0U;
        for (;;) {
            (void)FPGA_Debug_Write_LEDs(&hfpga_bridge, (beat % 2U) == 0U, false, false, 100U);
            if ((beat % 5U) == 0U) {
                HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
            }
            beat++;
            osDelay(100);
        }
    }

    /* Bind the FPGA IRQ event flag to this task handle (init_hardware ran without a task id) */
    IC_AttachTask(osThreadGetId(), IC_EVENT_FLAG_FPGA_IRQ);

    /* Release background tasks now that hardware passed POST */
    osThreadFlagsSet(TerminalTaskHandle, TASK_START_FLAG);
    osThreadFlagsSet(TemperatureTaskHandle, TASK_START_FLAG);
    osThreadFlagsSet(FreqGenTaskHandle, TASK_START_FLAG);

    FpgaDmaTelemetryPacket_t raw_telemetry;
    /* All-zero burst used as silent filler when no FPGA IRQ arrives */
    static const FpgaDmaTelemetryPacket_t silent_telemetry = {0};

    /* Main 400 Hz pipeline: wait FPGA IRQ -> resolver + ADC processing -> BC TX/RX */
    for (;;) {
        HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_SET); /* profiling pulse start */

        if (IC_Process(10)) {
            if (IC_GetRawTelemetry(&raw_telemetry)) {
                RDC_Process(&raw_telemetry);
                FPGA_ADC_Process(&raw_telemetry);
            }
            pspm_task_counter++;
        } else {
            /* No carrier: keep TX to BC alive with zeroed data (GLOBAL_ERROR asserted) */
            RDC_Process(&silent_telemetry);
            FPGA_ADC_Process(&silent_telemetry);
        }

        HOST_Process();
        host_process_rx_pending();
        host_rx_guard();
        led_indication_update();

        HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_RESET); /* profiling pulse end */
    }
}

/* DSPAssist (eAssist / DSPA) 1 ms periodic engine. Sleeps with 0% CPU
   consumption until authorized by app_default_task after POST. */
void app_terminal_task(void)
{
    osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, osWaitForever);

    /* 1 ms periodic DSPA engine execution loop */
    for (;;) {
        terminal_task();
    }
}

/* Background telemetry task: 10 Hz BMP280 pressure and STM32 die temperature. */
void app_temperature_task(void)
{
    static uint32_t last_wake_time = 0;

    osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, osWaitForever);
    last_wake_time = osKernelGetTickCount();

    /* Infinite loop: Deterministic 100 ms rate (10 Hz) */
    for (;;) {
        /* 1. Update Bosch BMP280 atmospheric pressure & temperature over SPI3 */
        BMP280_Process();

        /* 2. Update STM32 internal CPU core temperature and analog rail */
        Read_Temperature();

        last_wake_time += (100U * osKernelGetTickFreq()) / 1000U;
        osDelayUntil(last_wake_time);
    }
}

bool dbg_gen_dac_busy(void)
{
    return g_app_dbg_gen.mode;
}

/* 1 ms debug waveform generator: overrides one DAC channel (1..10) with the
   configured waveform while mode=1. At mode=0 the task idles on the tick
   grid only - no SPI traffic, no influence on the BC-controlled DAC outputs. */
void app_freq_generator_task(void)
{
    static uint32_t last_wake_time = 0U;

    osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, osWaitForever);
    last_wake_time = osKernelGetTickCount();
    CTestGenerator_init(&g_app_dbg_gen.gen, 1000.0f, 10.0f, T_W_NONE);

    for (;;) {
        /* Deterministic 1 ms grid without drift (osDelayUntil) */
        last_wake_time += (1U * osKernelGetTickFreq()) / 1000U;
        osDelayUntil(last_wake_time);

        if (!g_app_dbg_gen.mode) {
            continue;
        }

        float v = CTestGenerator_Process(&g_app_dbg_gen.gen, 0.001f);
        int32_t code = (v >= 0.0f) ? (int32_t)(v + 0.5f) : (int32_t)(v - 0.5f);
        if (code > 32767) {
            code = 32767;
        } else if (code < -32768) {
            code = -32768;
        }

        uint8_t ch = g_app_dbg_gen.dac_ch;
        if (ch < 1U || ch > 10U) {
            ch = 1U; /* terminal guard should prevent this; double safety net */
        }

        FpgaDmaDacPacket_t dac;
        memset(&dac, 0, sizeof(dac));
        dac.dac_val[ch - 1U] = (int16_t)code;
        (void)IC_WriteDacSetpoints(&dac, 10U);
    }
}
