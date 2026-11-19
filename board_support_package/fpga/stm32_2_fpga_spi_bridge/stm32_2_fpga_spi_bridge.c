/**
 ******************************************************************************
 * @file    stm32_2_fpga_spi_bridge.c
 * @brief   High-Speed SPI Driver Implementation for STM32 <-> FPGA Communication.
 *          Target MCU: STM32F411 (FreeRTOS CMSIS-OS v2 + HAL).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "stm32_2_fpga_spi_bridge.h"
#include <string.h>

/* Global handle pointer for ISR callbacks */
FPGA_HandleTypeDef *g_fpga_bridge = NULL;

/* ========================================================================= */
/*  PRIVATE MACROS                                                           */
/* ========================================================================= */

#define FPGA_CS_LOW(pin_ptr)   PIN_Reset_F(pin_ptr) /* Drives CS LOW  (Active)   */
#define FPGA_CS_HIGH(pin_ptr)  PIN_Set_F(pin_ptr)   /* Drives CS HIGH (Inactive) */

/* ========================================================================= */
/*  SECTION 1: INITIALIZATION                                                */
/* ========================================================================= */

//-----------------------------------------------------------------------------
// Initializes SPI bridge, binds dual CS pins, creates recursive mutex and sem
//-----------------------------------------------------------------------------
FPGA_Status_t	FPGA_Bridge_Init(FPGA_HandleTypeDef *hbridge,
                                 SPI_HandleTypeDef  *hspi,
                                 const Pin_Descriptor_t *nss_poll,
                                 const Pin_Descriptor_t *nss_dma){
	if(!hbridge || !hspi || !nss_poll || !nss_dma){
		return FPGA_ERROR;
	}

	hbridge->hspi           = hspi;
	hbridge->nss_poll_pin   = nss_poll;
	hbridge->nss_dma_pin    = nss_dma;
	hbridge->active_nss_pin = NULL;

	// Ensure BOTH CS lines are initially deselected (High-Z state for FPGA)
	FPGA_CS_HIGH(hbridge->nss_poll_pin);
	FPGA_CS_HIGH(hbridge->nss_dma_pin);

	// Recursive mutex prevents deadlocks during nested RMW operations
	const osMutexAttr_t mutex_attr = {
		.name       = "FpgaBridgeMutex",
		.attr_bits  = osMutexRecursive | osMutexPrioInherit,
		.cb_mem     = NULL,
		.cb_size    = 0U
	};

	hbridge->mutex_id = osMutexNew(&mutex_attr);
	if(hbridge->mutex_id == NULL){
		return FPGA_ERROR;
	}

	// Binary semaphore for DMA transfer completion (Initial count = 0)
	const osSemaphoreAttr_t sem_attr = {
		.name = "FpgaDmaSem"
	};
	hbridge->dma_sem_id = osSemaphoreNew(1, 0, &sem_attr);
	if(hbridge->dma_sem_id == NULL){
		return FPGA_ERROR;
	}

	// Assign global pointer for ISR callbacks
	g_fpga_bridge = hbridge;

	return FPGA_OK;
}

/* ========================================================================= */
/*  SECTION 2: CONTROL PLANE SYNCHRONOUS POLLING (NSS_P / PB0)               */
/* ========================================================================= */

//-----------------------------------------------------------------------------
// Synchronous 16-bit Register Write: Word 1 = Cmd(Bit15=1)+Addr, Word 2 = Data
//-----------------------------------------------------------------------------
FPGA_Status_t	FPGA_Write_Poll(FPGA_HandleTypeDef *hbridge, uint16_t addr, uint16_t data, uint32_t timeout_ms){
	HAL_StatusTypeDef hal_status;

	if(!hbridge) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
		return FPGA_BUSY;
	}

	// Word 1: Bit 15=1 (Write) | Address [9:0]
	hbridge->tx_poll_buf[0] = (1U << 15) | (addr & 0x03FFU);
	// Word 2: 16-bit Data payload
	hbridge->tx_poll_buf[1] = data;

	FPGA_CS_LOW(hbridge->nss_poll_pin);

	// In 16-bit SPI mode, Size = 2 sends 2 Half-Words (4 bytes)
	hal_status = HAL_SPI_TransmitReceive(hbridge->hspi,
	                                     (uint8_t*)hbridge->tx_poll_buf,
	                                     (uint8_t*)hbridge->rx_poll_buf,
	                                     2,
	                                     timeout_ms);

	FPGA_CS_HIGH(hbridge->nss_poll_pin);

	osMutexRelease(hbridge->mutex_id);

	if(hal_status == HAL_TIMEOUT) return FPGA_TIMEOUT;
	if(hal_status != HAL_OK)      return FPGA_ERROR;

	return FPGA_OK;
}

//-----------------------------------------------------------------------------
// Synchronous 16-bit Register Read: Word 1 = Cmd(Bit15=0)+Addr, Word 2 = Dummy
//-----------------------------------------------------------------------------
FPGA_Status_t	FPGA_Read_Poll(FPGA_HandleTypeDef *hbridge, uint16_t addr, uint16_t *data, uint32_t timeout_ms){
	HAL_StatusTypeDef hal_status;

	if(!hbridge || !data) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
		return FPGA_BUSY;
	}

	// Word 1: Bit 15=0 (Read) | Address [9:0]
	hbridge->tx_poll_buf[0] = (0U << 15) | (addr & 0x03FFU);
	// Word 2: Dummy word (0x0000)
	hbridge->tx_poll_buf[1] = 0x0000U;

	FPGA_CS_LOW(hbridge->nss_poll_pin);

	hal_status = HAL_SPI_TransmitReceive(hbridge->hspi,
	                                     (uint8_t*)hbridge->tx_poll_buf,
	                                     (uint8_t*)hbridge->rx_poll_buf,
	                                     2,
	                                     timeout_ms);

	FPGA_CS_HIGH(hbridge->nss_poll_pin);

	if(hal_status == HAL_OK){
		*data = hbridge->rx_poll_buf[1]; // Valid data returns in Word 2
	}

	osMutexRelease(hbridge->mutex_id);

	if(hal_status == HAL_TIMEOUT) return FPGA_TIMEOUT;
	if(hal_status != HAL_OK)      return FPGA_ERROR;

	return FPGA_OK;
}

/* ========================================================================= */
/*  SECTION 3: ATOMIC READ-MODIFY-WRITE BIT OPERATIONS                       */
/* ========================================================================= */

FPGA_Status_t	FPGA_Set_Bit_Poll(FPGA_HandleTypeDef *hbridge, uint16_t addr, uint16_t bit_mask, uint32_t timeout_ms){
	uint16_t      reg_val = 0;
	FPGA_Status_t status;

	if(!hbridge) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
		return FPGA_BUSY;
	}

	status = FPGA_Read_Poll(hbridge, addr, &reg_val, timeout_ms);
	if(status == FPGA_OK){
		reg_val |= bit_mask;
		status = FPGA_Write_Poll(hbridge, addr, reg_val, timeout_ms);
	}

	osMutexRelease(hbridge->mutex_id);
	return status;
}

FPGA_Status_t	FPGA_Clear_Bit_Poll(FPGA_HandleTypeDef *hbridge, uint16_t addr, uint16_t bit_mask, uint32_t timeout_ms){
	uint16_t      reg_val = 0;
	FPGA_Status_t status;

	if(!hbridge) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
		return FPGA_BUSY;
	}

	status = FPGA_Read_Poll(hbridge, addr, &reg_val, timeout_ms);
	if(status == FPGA_OK){
		reg_val &= ~bit_mask;
		status = FPGA_Write_Poll(hbridge, addr, reg_val, timeout_ms);
	}

	osMutexRelease(hbridge->mutex_id);
	return status;
}

FPGA_Status_t	FPGA_Toggle_Bit_Poll(FPGA_HandleTypeDef *hbridge, uint16_t addr, uint16_t bit_mask, uint32_t timeout_ms){
	uint16_t      reg_val = 0;
	FPGA_Status_t status;

	if(!hbridge) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
		return FPGA_BUSY;
	}

	status = FPGA_Read_Poll(hbridge, addr, &reg_val, timeout_ms);
	if(status == FPGA_OK){
		reg_val ^= bit_mask;
		status = FPGA_Write_Poll(hbridge, addr, reg_val, timeout_ms);
	}

	osMutexRelease(hbridge->mutex_id);
	return status;
}

FPGA_Status_t	FPGA_Modify_Bits_Poll(FPGA_HandleTypeDef *hbridge, uint16_t addr, uint16_t clear_mask, uint16_t set_mask, uint32_t timeout_ms){
	uint16_t      reg_val = 0;
	FPGA_Status_t status;

	if(!hbridge) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
		return FPGA_BUSY;
	}

	status = FPGA_Read_Poll(hbridge, addr, &reg_val, timeout_ms);
	if(status == FPGA_OK){
		reg_val = (reg_val & ~clear_mask) | set_mask;
		status  = FPGA_Write_Poll(hbridge, addr, reg_val, timeout_ms);
	}

	osMutexRelease(hbridge->mutex_id);
	return status;
}

FPGA_Status_t	FPGA_Is_Bit_Set_Poll(FPGA_HandleTypeDef *hbridge, uint16_t addr, uint16_t bit_mask, bool *is_set, uint32_t timeout_ms){
	uint16_t      reg_val = 0;
	FPGA_Status_t status;

	if(!hbridge || !is_set) return FPGA_ERROR;

	status = FPGA_Read_Poll(hbridge, addr, &reg_val, timeout_ms);
	if(status == FPGA_OK){
		*is_set = ((reg_val & bit_mask) != 0U);
	}
	return status;
}

/* ========================================================================= */
/*  SECTION 4: DATA PLANE DMA STREAMING (NSS_D / PB2)                        */
/* ========================================================================= */

//-----------------------------------------------------------------------------
// Reads 34-word Telemetry burst from FPGA via SPI1 DMA (544 bits in ~27.2 us)
// Size = 35 Half-Words (1 Command Header + 34 Telemetry Words)
//-----------------------------------------------------------------------------
FPGA_Status_t	FPGA_Read_Telemetry_DMA(FPGA_HandleTypeDef *hbridge,
                                       FpgaDmaTelemetryPacket_t *telemetry_out,
                                       uint32_t timeout_ms){
	HAL_StatusTypeDef hal_status;

	if(!hbridge || !telemetry_out) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK) return FPGA_BUSY;

	// Clear stale semaphore tokens
	while(osSemaphoreAcquire(hbridge->dma_sem_id, 0) == osOK);

	// Setup 1-word Command Header: Read Mode (0x2000)
	hbridge->dma_tx_frame.cmd_header = FPGA_DMA_CMD_READ;
	memset(&hbridge->dma_tx_frame.dac_payload, 0, sizeof(hbridge->dma_tx_frame.dac_payload));

	hbridge->active_nss_pin = hbridge->nss_dma_pin;
	FPGA_CS_LOW(hbridge->active_nss_pin);

	// Total transaction size = 34 Half-Words (1 Header + 33 Telemetry)
	hal_status = HAL_SPI_TransmitReceive_DMA(hbridge->hspi,
	                                        (uint8_t*)&hbridge->dma_tx_frame,
	                                        (uint8_t*)&hbridge->dma_rx_frame,
	                                        34);

	if(hal_status != HAL_OK){
		FPGA_CS_HIGH(hbridge->active_nss_pin);
		hbridge->active_nss_pin = NULL;
		osMutexRelease(hbridge->mutex_id);
		return FPGA_ERROR;
	}

	// Sleep calling thread until DMA ISR wakes it up
	if(osSemaphoreAcquire(hbridge->dma_sem_id, timeout_ms) != osOK){
		HAL_SPI_DMAStop(hbridge->hspi);
		FPGA_CS_HIGH(hbridge->active_nss_pin);
		hbridge->active_nss_pin = NULL;
		osMutexRelease(hbridge->mutex_id);
		return FPGA_TIMEOUT;
	}

	// Copy aligned telemetry payload (Words 1..34) directly to destination
	memcpy(telemetry_out, &hbridge->dma_rx_frame.telemetry, sizeof(FpgaDmaTelemetryPacket_t));

	osMutexRelease(hbridge->mutex_id);
	return FPGA_OK;
}

//-----------------------------------------------------------------------------
// Writes 10 DAC setpoints to FPGA via SPI1 DMA (160 bits in ~15 us)
// Size = 11 Half-Words (1 Header + 10 DACs)
//-----------------------------------------------------------------------------
FPGA_Status_t	FPGA_Write_DAC_DMA(FPGA_HandleTypeDef *hbridge,
                                   const FpgaDmaDacPacket_t *dac_in,
                                   uint32_t timeout_ms){
	HAL_StatusTypeDef hal_status;

	if(!hbridge || !dac_in) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK) return FPGA_BUSY;

	while(osSemaphoreAcquire(hbridge->dma_sem_id, 0) == osOK);

	// Setup 1-word Command Header: Write Mode (0x800A -> Length = 10)
	hbridge->dma_tx_frame.cmd_header = FPGA_DMA_CMD_WRITE;
	memcpy(&hbridge->dma_tx_frame.dac_payload, dac_in, sizeof(FpgaDmaDacPacket_t));

	hbridge->active_nss_pin = hbridge->nss_dma_pin;
	FPGA_CS_LOW(hbridge->active_nss_pin);

	// Total transaction size = 11 Half-Words (1 Header + 10 DACs)
	hal_status = HAL_SPI_Transmit_DMA(hbridge->hspi,
	                                 (uint8_t*)&hbridge->dma_tx_frame,
	                                 11);

	if(hal_status != HAL_OK){
		FPGA_CS_HIGH(hbridge->active_nss_pin);
		hbridge->active_nss_pin = NULL;
		osMutexRelease(hbridge->mutex_id);
		return FPGA_ERROR;
	}

	if(osSemaphoreAcquire(hbridge->dma_sem_id, timeout_ms) != osOK){
		HAL_SPI_DMAStop(hbridge->hspi);
		FPGA_CS_HIGH(hbridge->active_nss_pin);
		hbridge->active_nss_pin = NULL;
		osMutexRelease(hbridge->mutex_id);
		return FPGA_TIMEOUT;
	}

	osMutexRelease(hbridge->mutex_id);
	return FPGA_OK;
}

//-----------------------------------------------------------------------------
// Full-Duplex simultaneous DMA exchange: 10 DACs TX + 34 Telemetry words RX
// Size = 35 Half-Words
//-----------------------------------------------------------------------------
FPGA_Status_t	FPGA_Exchange_FullDuplex_DMA(FPGA_HandleTypeDef *hbridge,
                                            const FpgaDmaDacPacket_t *dac_in,
                                            FpgaDmaTelemetryPacket_t *telemetry_out,
                                            uint32_t timeout_ms){
	HAL_StatusTypeDef hal_status;

	if(!hbridge || !dac_in || !telemetry_out) return FPGA_ERROR;

	if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK) return FPGA_BUSY;

	while(osSemaphoreAcquire(hbridge->dma_sem_id, 0) == osOK);

	// Setup 1-word Command Header: Full-Duplex R/W Mode (0xC00A)
	hbridge->dma_tx_frame.cmd_header = FPGA_DMA_CMD_RW;
	memcpy(&hbridge->dma_tx_frame.dac_payload, dac_in, sizeof(FpgaDmaDacPacket_t));

	hbridge->active_nss_pin = hbridge->nss_dma_pin;
	FPGA_CS_LOW(hbridge->active_nss_pin);

	// Total transaction size = 35 Half-Words (1 Header + FPGA_DMA_READ_WORDS Telemetry)
	hal_status = HAL_SPI_TransmitReceive_DMA(hbridge->hspi,
	                                        (uint8_t*)&hbridge->dma_tx_frame,
	                                        (uint8_t*)&hbridge->dma_rx_frame,
	                                        FPGA_DMA_READ_WORDS + 1U);

	if(hal_status != HAL_OK){
		FPGA_CS_HIGH(hbridge->active_nss_pin);
		hbridge->active_nss_pin = NULL;
		osMutexRelease(hbridge->mutex_id);
		return FPGA_ERROR;
	}

	if(osSemaphoreAcquire(hbridge->dma_sem_id, timeout_ms) != osOK){
		HAL_SPI_DMAStop(hbridge->hspi);
		FPGA_CS_HIGH(hbridge->active_nss_pin);
		hbridge->active_nss_pin = NULL;
		osMutexRelease(hbridge->mutex_id);
		return FPGA_TIMEOUT;
	}

	memcpy(telemetry_out, &hbridge->dma_rx_frame.telemetry, sizeof(FpgaDmaTelemetryPacket_t));

	osMutexRelease(hbridge->mutex_id);
	return FPGA_OK;
}

/* ========================================================================= */
/*  SECTION 5: HAL SPI CALLBACKS (Interrupt Context)                         */
/* ========================================================================= */

static void _handle_dma_complete(SPI_HandleTypeDef *hspi){
	if(g_fpga_bridge != NULL && hspi == g_fpga_bridge->hspi){
		// CRITICAL: Wait for SPI shift register to finish transmitting the last bit!
		while(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY));

		if(g_fpga_bridge->active_nss_pin != NULL){
			FPGA_CS_HIGH(g_fpga_bridge->active_nss_pin);
			g_fpga_bridge->active_nss_pin = NULL;
		}

		osSemaphoreRelease(g_fpga_bridge->dma_sem_id);
	}
}

void	HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	_handle_dma_complete(hspi);
}

void	HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	_handle_dma_complete(hspi);
}

void	HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	_handle_dma_complete(hspi);
}
