/**
 ******************************************************************************
 * @file    app_tasks.h
 * @brief   PSPM application task bodies isolated from CubeMX-generated
 *          freertos.c. Task entry functions in freertos.c must remain
 *          one-line calls so that code regeneration can never destroy
 *          application logic.
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "cmsis_os.h"
#include "testgenerator.h"

/* Synchronization flag to release background tasks after hardware POST */
#define TASK_START_FLAG 0x01U

/* Debug waveform generator control (owned by app_freq_generator_task,
   edited live from the terminal sDBG subtree):
   mode   : false = generator idle (DAC left to BC), true = waveform active
   dac_ch : target DAC channel, 1..10 (all other channels output 0)
   gen    : waveform state; amplitude/offset are int16 DAC code units */
typedef struct {
    bool           mode;
    uint8_t        dac_ch;
    CTestGenerator gen;
} app_dbg_gen_cfg_t;

extern app_dbg_gen_cfg_t g_app_dbg_gen;

/* FreeRTOS thread handles defined by the generated freertos.c */
extern osThreadId_t TerminalTaskHandle;
extern osThreadId_t TemperatureTaskHandle;
extern osThreadId_t FreqGenTaskHandle;

/* Global 400 Hz control loop execution counter (shown in terminal) */
extern uint32_t pspm_task_counter;

/* Task bodies, called once from the matching Start*Task entry in freertos.c */
void app_default_task(void);
void app_terminal_task(void);
void app_temperature_task(void);
void app_freq_generator_task(void);

/* true = debug generator currently owns the DAC outputs (BC writes skipped) */
bool dbg_gen_dac_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H_ */
