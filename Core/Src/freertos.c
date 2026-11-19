/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Real-Time Task Sequencer for PSPM Combat System.
  *                      Deterministic 400 Hz FPGA Loop & Background Tasks.
  *                      Target MCU: STM32F411RET6 (100 MHz / FreeRTOS CMSIS-OS v2).
  *                      All comments in pure ASCII English.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#include "app_tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Application task bodies live in board_support_package/app_tasks.c so that
   CubeMX regeneration of this file can never destroy application logic. */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TerminalTask */
osThreadId_t TerminalTaskHandle;
const osThreadAttr_t TerminalTask_attributes = {
  .name = "TerminalTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TemperatureTask */
osThreadId_t TemperatureTaskHandle;
const osThreadAttr_t TemperatureTask_attributes = {
  .name = "TemperatureTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FreqGenTask */
osThreadId_t FreqGenTaskHandle;
const osThreadAttr_t FreqGenTask_attributes = {
  .name = "FreqGenTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTerminalTask(void *argument);
void StartTemperaturePressureTask(void *argument);
void StartFreqGeneratorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TerminalTask */
  TerminalTaskHandle = osThreadNew(StartTerminalTask, NULL, &TerminalTask_attributes);

  /* creation of TemperatureTask */
  TemperatureTaskHandle = osThreadNew(StartTemperaturePressureTask, NULL, &TemperatureTask_attributes);

  /* creation of FreqGenTask */
  FreqGenTaskHandle = osThreadNew(StartFreqGeneratorTask, NULL, &FreqGenTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Default Task: Master 400 Hz Real-Time Control Sequencer (T = 2.500 ms).
  * @note   Pipeline per period:
  *          1. Sleeps on EXTI10 hardware notification (IC_Process).
  *          2. Fetches 34-word RAW telemetry burst via SPI1 DMA (NSS_D / PB2).
  *          3. Executes Type-II PLL Tracking Filter for Resolver (RDC_Process).
  *          4. Computes Peak Demodulated ADC Scaling (FPGA_ADC_Process).
  *          5. Assembles and starts non-blocking 32-byte UART2 DMA TX to BC (HOST_Process).
  *          6. Drains BC UART RX ring and runs RX silence watchdog (host_process_rx_pending / host_rx_guard).
  *          7. Profiling: Generates active-high pulse on TP1 for oscilloscope metrics.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  app_default_task();
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTerminalTask */
/**
  * @brief  Terminal Task: DSPAssist (eAssist / DSPA) 1 ms Periodic Engine.
  * @note   Sleeps with 0% CPU consumption until authorized by defaultTask.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTerminalTask */
void StartTerminalTask(void *argument)
{
  /* USER CODE BEGIN StartTerminalTask */
  app_terminal_task();
  /* USER CODE END StartTerminalTask */
}

/* USER CODE BEGIN Header_StartTemperaturePressureTask */
/**
  * @brief  Background Telemetry Task: 10 Hz Periodic Execution Loop.
  * @note   Updates Bosch BMP280 atmospheric pressure and internal STM32 CPU temperature.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTemperaturePressureTask */
void StartTemperaturePressureTask(void *argument)
{
  /* USER CODE BEGIN StartTemperaturePressureTask */
  app_temperature_task();
  /* USER CODE END StartTemperaturePressureTask */
}

/* USER CODE BEGIN Header_StartFreqGeneratorTask */
/**
* @brief Function implementing the FreqGeneratorTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFreqGeneratorTask */
void StartFreqGeneratorTask(void *argument)
{
  /* USER CODE BEGIN StartFreqGeneratorTask */
  app_freq_generator_task();
  /* USER CODE END StartFreqGeneratorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

