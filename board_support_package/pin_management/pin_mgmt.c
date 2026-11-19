/**
 ******************************************************************************
 * @file    pin_mgmt.c
 * @brief   Pin Management & Dual-Plane Bus Arbitration Library Implementation.
 *          Target MCU: STM32F411RET6 (100 MHz, FreeRTOS CMSIS-OS v2).
 *          Board: PSPM (DIAS.901.02.00.300).
 *          All comments in pure ASCII English.
 ******************************************************************************
 */

#include "pin_mgmt.h"

/* Global Pin Management Configuration */
Pin_Mgmt_Config_t bsp_pin_config = PIN_MGMT_CONFIG_DEFAULT;

static uint8_t      spi_bus_acquired = 0;
static osMutexId_t  gpio_mutex       = NULL;

/* ========================================================================= */
/*  PHYSICAL PIN DESCRIPTORS INITIALIZATION                                  */
/* ========================================================================= */

const Pin_Descriptor_t pin_led_green = PIN_DESC(LED_GREEN_GPIO_Port, LED_GREEN_Pin, "LED_GREEN");

/* pin_clk25mhz FORBIDDEN: PH0 is the HSE 25 MHz clock input (RCC_OSC_IN).
   Never reconfigure as GPIO; the PLL runs from this clock. */
// const Pin_Descriptor_t pin_clk25mhz = PIN_DESC(CLK25MHZ_GPIO_Port, CLK25MHZ_Pin, "CLK25MHZ");

/* Test Points for Logic Analyzer / Oscilloscope Profiling */
const Pin_Descriptor_t pin_tp1        = PIN_DESC(TP1_GPIO_Port, TP1_Pin, "TP1");
const Pin_Descriptor_t pin_tp2        = PIN_DESC(TP2_GPIO_Port, TP2_Pin, "TP2");
const Pin_Descriptor_t pin_tp3        = PIN_DESC(TP3_GPIO_Port, TP3_Pin, "TP3");
const Pin_Descriptor_t pin_tp4        = PIN_DESC(TP4_GPIO_Port, TP4_Pin, "TP4");

/* FPGA Boot Status Pin (PA1) */
const Pin_Descriptor_t pin_fpga_done  = PIN_DESC(DONE_GPIO_Port, DONE_Pin, "FPGA_DONE");

/* UART2: Link to FPGA UART Multiplexer */
const Pin_Descriptor_t pin_usart2_fpga_tx = PIN_DESC(USART2_FPGA_TX_GPIO_Port, USART2_FPGA_TX_Pin, "USART2_FPGA_TX");
const Pin_Descriptor_t pin_usart2_fpga_rx = PIN_DESC(USART2_FPGA_RX_GPIO_Port, USART2_FPGA_RX_Pin, "USART2_FPGA_RX");

/* PA4: Flash W25Q128 Chip Select (Kept as Input in MainAppl) */
const Pin_Descriptor_t pin_flash_cs = PIN_DESC(SP1_FPGA_CSO_GPIO_Port, SP1_FPGA_CSO_Pin, "FLASH_CS");

/* PSPM Dual-Plane Chip Select Pins */
const Pin_Descriptor_t pin_stm32_2_fpga_nss_p = PIN_DESC(STM32_2_FPGA_NSS_P_GPIO_Port, STM32_2_FPGA_NSS_P_Pin, "FPGA_NSS_P");
const Pin_Descriptor_t pin_fpga_cs            = PIN_DESC(STM32_2_FPGA_NSS_P_GPIO_Port, STM32_2_FPGA_NSS_P_Pin, "FPGA_NSS_P");
const Pin_Descriptor_t pin_stm32_2_fpga_nss_d = PIN_DESC(STM32_2_FPGA_NSS_D_GPIO_Port, STM32_2_FPGA_NSS_D_Pin, "FPGA_NSS_D");
const Pin_Descriptor_t pin_interrupt_fpga     = PIN_DESC(INTERRUPT_FPGA_GPIO_Port, INTERRUPT_FPGA_Pin, "INTERRUPT_FPGA");

/* SPI1 Data Bus (PA5, PA6, PA7) */
const Pin_Descriptor_t pin_spi_sck   = PIN_DESC(SP1_FPGA_SCK_GPIO_Port, SP1_FPGA_SCK_Pin, "SPI1_SCK");
const Pin_Descriptor_t pin_spi_miso  = PIN_DESC(SPI1_FPGA_MISO_GPIO_Port, SPI1_FPGA_MISO_Pin, "SPI1_MISO");
const Pin_Descriptor_t pin_spi_mosi  = PIN_DESC(SPI1_FPGA_MOSI_GPIO_Port, SPI1_FPGA_MOSI_Pin, "SPI1_MOSI");

/* Dedicated SPI3 Chip Select for Bosch BMP280 (PC10) */
const Pin_Descriptor_t pin_spi3_cso  = PIN_DESC(SPI3_CSO_GPIO_Port, SPI3_CSO_Pin, "BMP280_CS");

/* Miscellaneous and Reset Pins */
const Pin_Descriptor_t pin_fpga_2_stm32_misc1 = PIN_DESC(FPGA_2_STM32_MISC1_GPIO_Port, FPGA_2_STM32_MISC1_Pin, "FPGA_MISC1");
const Pin_Descriptor_t pin_mr_fpga   = PIN_DESC(MR_FPGA_GPIO_Port, MR_FPGA_Pin, "MR_FPGA");
const Pin_Descriptor_t pin_mr_prog   = PIN_DESC(MR_PROG_GPIO_Port, MR_PROG_Pin, "MR_PROG"); /* PB15: PROG_B */

/* UART6: Debug / ST-Link VCP */
const Pin_Descriptor_t pin_usart6_debug_tx = PIN_DESC(USART6_DEBUG_TX_GPIO_Port, USART6_DEBUG_TX_Pin, "USART6_DEBUG_TX");
const Pin_Descriptor_t pin_usart6_debug_rx = PIN_DESC(USART6_DEBUG_RX_GPIO_Port, USART6_DEBUG_RX_Pin, "USART6_DEBUG_RX");

/* UART1: KPA / RS-485 */
const Pin_Descriptor_t pin_usart1_kpa_te = PIN_DESC(USART1_KPA_TE_GPIO_Port, USART1_KPA_TE_Pin, "USART1_KPA_TE");
const Pin_Descriptor_t pin_usart1_kpa_tx = PIN_DESC(USART1_KPA_TX_GPIO_Port, USART1_KPA_TX_Pin, "USART1_KPA_TX");
const Pin_Descriptor_t pin_usart1_kpa_rx = PIN_DESC(USART1_KPA_RX_GPIO_Port, USART1_KPA_RX_Pin, "USART1_KPA_RX");

/* ST-Link Hardware Detection Pin (PB7) */
const Pin_Descriptor_t pin_stlink_detect = PIN_DESC(STLINK_GND_TEST_GPIO_Port, STLINK_GND_TEST_Pin, "STLINK_DETECT");

/* ========================================================================= */
/*  PRIVATE FUNCTIONS                                                        */
/* ========================================================================= */

/**
 * @brief  Creates recursive GPIO mutex for thread-safe multi-task access.
 * @note   Uses osMutexRecursive and osMutexPrioInherit attributes.
 * @retval osOK on success, osErrorResource on memory allocation failure.
 */
static osStatus_t _create_gpio_mutex(void){
    if(gpio_mutex == NULL){
        const osMutexAttr_t mutex_attr = {
            .name       = "PIN_MGMT_GPIO_Mutex",
            .attr_bits  = osMutexRecursive | osMutexPrioInherit,
            .cb_mem     = NULL,
            .cb_size    = 0U
        };
        gpio_mutex = osMutexNew(&mutex_attr);
        if(gpio_mutex == NULL) return osErrorResource;
    }
    return osOK;
}

/**
 * @brief  Deletes the FreeRTOS GPIO mutex.
 * @retval osOK on success.
 */
static osStatus_t _delete_gpio_mutex(void){
    if(gpio_mutex != NULL){
        osMutexDelete(gpio_mutex);
        gpio_mutex = NULL;
    }
    return osOK;
}

/**
 * @brief  Configures SPI1 lines to AF5 Master mode and enables Dual Chip Selects (PB0/PB2).
 * @note   PA5 (SCK), PA6 (MISO), PA7 (MOSI) -> Alternate Function 5 (SPI1, VERY_HIGH speed).
 *         PB0 (NSS_P) and PB2 (NSS_D)       -> Push-Pull Outputs, initialized High (1).
 * @retval osOK on success.
 */
static osStatus_t _spi_bus_af_mode(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SCK (PA5), MISO (PA6), MOSI (PA7) -> AF5 Master */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

    GPIO_InitStruct.Pin = pin_spi_sck.pin;
    HAL_GPIO_Init(pin_spi_sck.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pin_spi_miso.pin;
    HAL_GPIO_Init(pin_spi_miso.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pin_spi_mosi.pin;
    HAL_GPIO_Init(pin_spi_mosi.port, &GPIO_InitStruct);

    /* FPGA Polling CS (PB0) -> Output Push-Pull, Initial HIGH (1) */
    GPIO_InitStruct.Pin   = pin_stm32_2_fpga_nss_p.pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(pin_stm32_2_fpga_nss_p.port, &GPIO_InitStruct);
    PIN_Set_F(&pin_stm32_2_fpga_nss_p);

    /* FPGA DMA Burst CS (PB2) -> Output Push-Pull, Initial HIGH (1) */
    GPIO_InitStruct.Pin   = pin_stm32_2_fpga_nss_d.pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(pin_stm32_2_fpga_nss_d.port, &GPIO_InitStruct);
    PIN_Set_F(&pin_stm32_2_fpga_nss_d);

    return osOK;
}

/**
 * @brief  Puts SPI1 lines and Chip Selects into Analog (Hi-Z) mode.
 * @note   Used during FPGA power-up configuration to prevent bus contention while FPGA reads Flash.
 * @retval osOK on success.
 */
static osStatus_t _spi_bus_hiz_mode(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = pin_spi_sck.pin;
    HAL_GPIO_Init(pin_spi_sck.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pin_spi_miso.pin;
    HAL_GPIO_Init(pin_spi_miso.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pin_spi_mosi.pin;
    HAL_GPIO_Init(pin_spi_mosi.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pin_stm32_2_fpga_nss_p.pin;
    HAL_GPIO_Init(pin_stm32_2_fpga_nss_p.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = pin_stm32_2_fpga_nss_d.pin;
    HAL_GPIO_Init(pin_stm32_2_fpga_nss_d.port, &GPIO_InitStruct);

    return osOK;
}

/* ========================================================================= */
/*  PUBLIC API IMPLEMENTATION                                                */
/* ========================================================================= */

/**
 * @brief  Initializes pin management subsystem, resets acquisition flag, and creates mutex.
 * @param  config: Pointer to Pin_Mgmt_Config_t (or NULL for defaults).
 * @retval osOK on success, error code otherwise.
 */
osStatus_t PIN_MGMT_Init(Pin_Mgmt_Config_t *config){
    if(config != NULL){
        bsp_pin_config = *config;
    }
    spi_bus_acquired = 0;
    return _create_gpio_mutex();
}

/**
 * @brief  Deinitializes pin management subsystem and deletes mutex.
 * @retval osOK on success.
 */
osStatus_t PIN_MGMT_DeInit(void){
    _delete_gpio_mutex();
    return osOK;
}

/**
 * @brief  Sets GPIO pin state to High (Thread-safe with mutex protection).
 * @param  pin: Pointer to Pin_Descriptor_t.
 * @retval osOK on success, error code otherwise.
 */
osStatus_t PIN_Set(const Pin_Descriptor_t *pin){
    if(!pin || !pin->port) return osErrorParameter;
    osStatus_t status = PIN_GPIO_Mutex_Acquire(osWaitForever);
    if(status != osOK) return status;

    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_SET);
    PIN_GPIO_Mutex_Release();
    return osOK;
}

/**
 * @brief  Sets GPIO pin state to Low (Thread-safe with mutex protection).
 * @param  pin: Pointer to Pin_Descriptor_t.
 * @retval osOK on success, error code otherwise.
 */
osStatus_t PIN_Reset(const Pin_Descriptor_t *pin){
    if(!pin || !pin->port) return osErrorParameter;
    osStatus_t status = PIN_GPIO_Mutex_Acquire(osWaitForever);
    if(status != osOK) return status;

    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_RESET);
    PIN_GPIO_Mutex_Release();
    return osOK;
}

/**
 * @brief  Toggles GPIO pin state (Thread-safe with mutex protection).
 * @param  pin: Pointer to Pin_Descriptor_t.
 * @retval osOK on success, error code otherwise.
 */
osStatus_t PIN_Toggle(const Pin_Descriptor_t *pin){
    if(!pin || !pin->port) return osErrorParameter;
    osStatus_t status = PIN_GPIO_Mutex_Acquire(osWaitForever);
    if(status != osOK) return status;

    HAL_GPIO_TogglePin(pin->port, pin->pin);
    PIN_GPIO_Mutex_Release();
    return osOK;
}

/**
 * @brief  Reads GPIO pin physical state (Thread-safe with mutex protection).
 * @param  pin: Pointer to Pin_Descriptor_t.
 * @retval 1 if pin is High, 0 if Low.
 */
uint8_t PIN_Read(const Pin_Descriptor_t *pin){
    if(!pin || !pin->port) return 0;
    uint8_t state = 0;
    if(PIN_GPIO_Mutex_Acquire(osWaitForever) == osOK){
        state = (HAL_GPIO_ReadPin(pin->port, pin->pin) == GPIO_PIN_SET) ? 1 : 0;
        PIN_GPIO_Mutex_Release();
    }
    return state;
}

/* ========================================================================= */
/*  FAST ATOMIC REGISTER OPERATIONS (Zero RTOS Overhead)                     */
/* ========================================================================= */

/**
 * @brief  Fast atomic pin SET via direct BSRR register write.
 * @note   Sub-microsecond execution, zero RTOS mutex overhead.
 * @param  pin: Pointer to Pin_Descriptor_t.
 * @retval osOK on success.
 */
osStatus_t PIN_Set_F(const Pin_Descriptor_t *pin){
    if(!pin || !pin->port) return osErrorParameter;
    pin->port->BSRR = pin->pin;
    return osOK;
}

/**
 * @brief  Fast atomic pin RESET via upper BSRR register write.
 * @note   Sub-microsecond execution, zero RTOS mutex overhead.
 * @param  pin: Pointer to Pin_Descriptor_t.
 * @retval osOK on success.
 */
osStatus_t PIN_Reset_F(const Pin_Descriptor_t *pin){
    if(!pin || !pin->port) return osErrorParameter;
    pin->port->BSRR = (uint32_t)pin->pin << 16U;
    return osOK;
}

/**
 * @brief  Fast atomic pin TOGGLE via BSRR register.
 * @note   Prevents Read-Modify-Write race conditions on concurrent GPIO ports.
 * @param  pin: Pointer to Pin_Descriptor_t.
 * @retval osOK on success.
 */
osStatus_t PIN_Toggle_F(const Pin_Descriptor_t *pin){
    if(!pin || !pin->port) return osErrorParameter;
    uint32_t odr = pin->port->ODR;
    if(odr & pin->pin){
        pin->port->BSRR = (uint32_t)pin->pin << 16U;
    } else {
        pin->port->BSRR = pin->pin;
    }
    return osOK;
}

/**
 * @brief  Fast direct non-blocking read of physical IDR register.
 * @param  pin: Pointer to Pin_Descriptor_t.
 * @retval 1 if pin is High, 0 if Low.
 */
uint8_t PIN_Read_F(const Pin_Descriptor_t *pin){
    if(!pin || !pin->port) return 0;
    return (pin->port->IDR & pin->pin) ? 1 : 0;
}

/* ========================================================================= */
/*  GPIO MUTEX CONTROLS                                                      */
/* ========================================================================= */

/**
 * @brief  Acquires the recursive GPIO mutex with timeout.
 * @param  timeout: Timeout in OS ticks or osWaitForever.
 * @retval osOK on success.
 */
osStatus_t PIN_GPIO_Mutex_Acquire(uint32_t timeout){
    if(gpio_mutex == NULL){
        osStatus_t status = _create_gpio_mutex();
        if(status != osOK) return status;
    }
    return osMutexAcquire(gpio_mutex, timeout);
}

/**
 * @brief  Releases the recursive GPIO mutex.
 * @retval osOK on success.
 */
osStatus_t PIN_GPIO_Mutex_Release(void){
    if(gpio_mutex == NULL) return osErrorResource;
    return osMutexRelease(gpio_mutex);
}

/**
 * @brief  Non-blocking check if the GPIO mutex is currently locked.
 * @retval 1 if locked, 0 if free.
 */
uint8_t PIN_GPIO_Mutex_Is_Locked(void){
    if(gpio_mutex == NULL) return 0;
    osStatus_t status = osMutexAcquire(gpio_mutex, 0);
    if(status == osOK){
        osMutexRelease(gpio_mutex);
        return 0;
    }
    return 1;
}

/* ========================================================================= */
/*  FPGA BRING-UP & STATUS SYNCHRONIZATION                                   */
/* ========================================================================= */

/**
 * @brief  Polls the FPGA DONE pin (PA1) until it rises to High (1) with timeout.
 * @note   Universal helper used after hardware reset and power-up configuration.
 * @param  timeout_ms: Maximum wait duration in milliseconds.
 * @retval osOK if DONE became High, osErrorTimeout if FPGA failed to configure.
 */
osStatus_t FPGA_Wait_Ready(uint32_t timeout_ms){
    uint32_t start_tick = HAL_GetTick();
    while(!FPGA_Is_Ready()){
        if((HAL_GetTick() - start_tick) > timeout_ms){
            return osErrorTimeout;
        }
        PIN_Delay(10);
    }
    return osOK;
}

/**
 * @brief  Fast direct read of FPGA DONE status line (PA1).
 * @retval 1 if FPGA is configured (DONE=1), 0 if booting or unprogrammed.
 */
uint8_t FPGA_Is_Ready(void){
    return PIN_Read_F(&pin_fpga_done);
}

/**
 * @brief  Checks whether FPGA is currently being held in hardware reset.
 * @retval 1 if MR_PROG line is Low (Reset), 0 if released.
 */
uint8_t FPGA_Is_In_Reset(void){
    return (PIN_Read_F(&pin_mr_prog) == 0) ? 1 : 0;
}

/**
 * @brief  Full hardware bring-up reset pulse for FPGA with DONE polling.
 * @note   1. Asserts MR_PROG Low (PB15) for reset_time_ms.
 *         2. Releases MR_PROG High.
 *         3. Delegates waiting to FPGA_Wait_Ready(timeout_ms).
 * @param  reset_time_ms: Reset pulse width in ms (typically 10 ms).
 * @param  timeout_ms: Maximum wait timeout in ms (MSB uses 1500 ms).
 * @retval osOK on success, osErrorTimeout on failure.
 */
osStatus_t FPGA_Reset_With_Check(uint32_t reset_time_ms, uint32_t timeout_ms){
    PIN_Reset_F(&pin_mr_prog);
    PIN_Delay(reset_time_ms);
    PIN_Set_F(&pin_mr_prog);
    return FPGA_Wait_Ready(timeout_ms);
}

/* ========================================================================= */
/*  SPI BUS ARBITRATION                                                      */
/* ========================================================================= */

/**
 * @brief  Configures physical mode of SPI1 and Chip Select pins.
 * @param  mode: SPI_BUS_AF_MODE (Active Master) or SPI_BUS_HIZ_MODE (Hi-Z Analog).
 * @retval osOK on success.
 */
osStatus_t SPI_Bus_Configure(SPI_Bus_Mode_t mode){
    switch(mode){
        case SPI_BUS_AF_MODE:  return _spi_bus_af_mode();
        case SPI_BUS_HIZ_MODE: return _spi_bus_hiz_mode();
        default:               return osErrorParameter;
    }
}

/**
 * @brief  Thread-safely reclaims SPI1 bus back to STM32 Master mode (AF5).
 * @retval osOK on success.
 */
osStatus_t SPI_Bus_Acquire_For_STM32(void){
    osStatus_t status = PIN_GPIO_Mutex_Acquire(osWaitForever);
    if(status != osOK) return status;

    if(spi_bus_acquired){
        PIN_GPIO_Mutex_Release();
        return osOK;
    }

    status = SPI_Bus_Configure(SPI_BUS_AF_MODE);
    if(status == osOK){
        spi_bus_acquired = 1;
    }
    PIN_GPIO_Mutex_Release();
    return status;
}

/**
 * @brief  Thread-safely releases SPI1 bus into Hi-Z Analog mode for FPGA boot.
 * @retval osOK on success.
 */
osStatus_t SPI_Bus_Release_To_FPGA(void){
    osStatus_t status = PIN_GPIO_Mutex_Acquire(osWaitForever);
    if(status != osOK) return status;

    if(!spi_bus_acquired){
        PIN_GPIO_Mutex_Release();
        return osOK;
    }

    status = SPI_Bus_Configure(SPI_BUS_HIZ_MODE);
    if(status == osOK){
        spi_bus_acquired = 0;
    }
    PIN_GPIO_Mutex_Release();
    return status;
}

/**
 * @brief  Checks whether SPI1 bus is currently acquired in STM32 Master mode.
 * @retval 1 if acquired, 0 if released to Hi-Z.
 */
uint8_t SPI_Bus_Is_Acquired(void){
    uint8_t acquired = 0;
    if(PIN_GPIO_Mutex_Acquire(10) == osOK){
        acquired = spi_bus_acquired;
        PIN_GPIO_Mutex_Release();
    }
    return acquired;
}

/* ========================================================================= */
/*  DIAGNOSTICS & SYSTEM UTILITIES                                           */
/* ========================================================================= */

/**
 * @brief  Checks if ST-Link hardware debugger probe is attached (PB7 pulled Low).
 * @retval 1 if ST-Link is connected, 0 if standalone field operation.
 */
uint8_t STLINK_Is_Connected(void){
    return (PIN_Read_F(&pin_stlink_detect) == 0) ? 1 : 0;
}

/**
 * @brief  Universal delay function that yields RTOS task or calls HAL_Delay.
 * @param  ms: Delay duration in milliseconds.
 * @retval osOK on completion.
 */
osStatus_t PIN_Delay(uint32_t ms){
    if(osKernelGetState() == osKernelRunning){
        return osDelay(ms);
    } else {
        HAL_Delay(ms);
        return osOK;
    }
}

/**
 * @brief  Visual diagnostic blinking helper for designated pin.
 * @param  pin: Target pin descriptor.
 * @param  count: Number of blink pulses.
 * @param  delay_ms: Duration of each ON and OFF phase in ms.
 * @retval osOK on success.
 */
osStatus_t PIN_Blink(const Pin_Descriptor_t *pin, uint8_t count, uint32_t delay_ms){
    if(!pin) return osErrorParameter;
    for(uint8_t i = 0; i < count; i++){
        PIN_Set(pin);
        PIN_Delay(delay_ms);
        PIN_Reset(pin);
        PIN_Delay(delay_ms);
    }
    return osOK;
}
