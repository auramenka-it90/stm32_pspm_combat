/**
 ******************************************************************************
 * @file    fpga_control.c
 * @brief   High-Level FPGA Module Control API Implementation for PSPM.
 *          Provides Thread-Safe RMW access to Debug & Configuration registers.
 *          Target: Xilinx Spartan-6 + STM32F4 (SPI Mode 00).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "fpga_control.h"

/* ========================================================================= */
/*  DEBUG MODULE APIS (Device ID = 1)                                        */
/* ========================================================================= */

/**
 * @brief  Verifies FPGA alive status by reading hardcoded constant 0xDEAD.
 * @note   Target: Device ID 1, Reg 0x01 (REG_S_DEBUG_CONST).
 *         Returns error on SPI bus fault, clock noise, or incorrect bitstream.
 */
FPGA_Status_t FPGA_Debug_Verify(FPGA_HandleTypeDef *hbridge, uint32_t timeout_ms){
    uint16_t const_val = 0;
    FPGA_Status_t status;

    status = FPGA_Read_Poll(hbridge, ADDR_S_DEBUG_CONST, &const_val, timeout_ms);
    if(status != FPGA_OK){
        return status;
    }

    if(const_val != FPGA_DEBUG_CONST_VAL){
        return FPGA_ERROR; // Signature mismatch (Clock noise, bad bitstream, or wrong CS)
    }

    return FPGA_OK;
}

/**
 * @brief  Executes full-duplex SPI bus loopback test using scratchpad register.
 * @note   Target: Device ID 1, Reg 0x00 (REG_S_DEBUG_FEEDBACK).
 *         Writes test pattern and reads it back to verify line integrity.
 */
FPGA_Status_t FPGA_Debug_Test_Echo(FPGA_HandleTypeDef *hbridge, uint16_t test_val, uint32_t timeout_ms){
    FPGA_Status_t status;
    uint16_t readback_val = 0;

    status = FPGA_Write_Poll(hbridge, ADDR_S_DEBUG_FEEDBACK, test_val, timeout_ms);
    if(status != FPGA_OK){
        return status;
    }

    status = FPGA_Read_Poll(hbridge, ADDR_S_DEBUG_FEEDBACK, &readback_val, timeout_ms);
    if(status != FPGA_OK){
        return status;
    }

    if(readback_val != test_val){
        return FPGA_ERROR; // Bus loopback integrity check failed
    }

    return FPGA_OK;
}

/**
 * @brief  Updates on-board status LEDs (VD35..37) via thread-safe RMW.
 * @note   Target: Device ID 1, Reg 0x02, Bits [2:0].
 *         Bit 0 = Green (VD35), Bit 1 = Yellow (VD36), Bit 2 = Red (VD37).
 */
FPGA_Status_t FPGA_Debug_Set_LEDs(FPGA_HandleTypeDef *hbridge, uint8_t led_mask, uint32_t timeout_ms){
    FPGA_Status_t status;
    uint16_t reg_val = 0;

    if(!hbridge) return FPGA_ERROR;

    // Recursive mutex allows safe nested acquisition inside Read/Write functions
    if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
        return FPGA_BUSY;
    }

    status = FPGA_Read_Poll(hbridge, ADDR_S_DEBUG_MISC, &reg_val, timeout_ms);
    if(status == FPGA_OK){
        reg_val &= ~FPGA_MISC_LED_MASK;
        reg_val |= ((uint16_t)led_mask & FPGA_MISC_LED_MASK);
        status = FPGA_Write_Poll(hbridge, ADDR_S_DEBUG_MISC, reg_val, timeout_ms);
    }

    osMutexRelease(hbridge->mutex_id);
    return status;
}

/**
 * @brief  Direct boolean control helper for Red, Yellow, and Green LEDs.
 * @param  r: Red LED (VD37), y: Yellow LED (VD36), g: Green LED (VD35).
 */
FPGA_Status_t FPGA_Debug_Write_LEDs(FPGA_HandleTypeDef *hbridge, bool r, bool y, bool g, uint32_t timeout_ms){
    uint8_t led_mask = 0;

    if(r) led_mask |= FPGA_MISC_LED_RED;
    if(y) led_mask |= FPGA_MISC_LED_YELLOW;
    if(g) led_mask |= FPGA_MISC_LED_GREEN;

    return FPGA_Debug_Set_LEDs(hbridge, led_mask, timeout_ms);
}

/**
 * @brief  Switches STM32 USART2 routing to target RS-485 transceiver.
 * @note   Target: Device ID 1, Reg 0x02, Bit 3 (0 = DD4 Channel 1, 1 = DD5 Channel 2).
 */
FPGA_Status_t FPGA_Debug_Set_UART_Mux(FPGA_HandleTypeDef *hbridge, FPGA_Uart_Port_t port, uint32_t timeout_ms){
    FPGA_Status_t status;
    uint16_t reg_val = 0;

    if(!hbridge) return FPGA_ERROR;

    if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
        return FPGA_BUSY;
    }

    status = FPGA_Read_Poll(hbridge, ADDR_S_DEBUG_MISC, &reg_val, timeout_ms);
    if(status == FPGA_OK){
        reg_val &= ~FPGA_MISC_UART_MUX_MASK;
        if(port == FPGA_UART_PORT_DD5){
            reg_val |= FPGA_MISC_UART_MUX_DD5;
        }
        status = FPGA_Write_Poll(hbridge, ADDR_S_DEBUG_MISC, reg_val, timeout_ms);
    }

    osMutexRelease(hbridge->mutex_id);
    return status;
}

/**
 * @brief  Selects reference phase comparator for Resolver demodulation.
 * @note   Target: Device ID 1, Reg 0x02, Bit 4 (0 = UON12 0 deg, 1 = UON23 120 deg).
 */
FPGA_Status_t FPGA_Debug_Set_Resolver_Phase(FPGA_HandleTypeDef *hbridge, FPGA_Resolver_Phase_t phase, uint32_t timeout_ms){
    FPGA_Status_t status;
    uint16_t reg_val = 0;

    if(!hbridge) return FPGA_ERROR;

    if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
        return FPGA_BUSY;
    }

    status = FPGA_Read_Poll(hbridge, ADDR_S_DEBUG_MISC, &reg_val, timeout_ms);
    if(status == FPGA_OK){
        reg_val &= ~FPGA_MISC_RES_PHASE_MASK;
        if(phase == FPGA_RES_PHASE_UON23){
            reg_val |= FPGA_MISC_RES_PHASE_U23;
        }
        status = FPGA_Write_Poll(hbridge, ADDR_S_DEBUG_MISC, reg_val, timeout_ms);
    }

    osMutexRelease(hbridge->mutex_id);
    return status;
}

/**
 * @brief  Enables/Disables internal mathematical 400 Hz 3-Phase BIST generator.
 * @note   Target: Device ID 1, Reg 0x02, Bit 5 (0 = Ext Pins, 1 = Internal Gen).
 */
FPGA_Status_t FPGA_Debug_Set_BIST_Enable(FPGA_HandleTypeDef *hbridge, bool enable, uint32_t timeout_ms){
    FPGA_Status_t status;
    uint16_t reg_val = 0;

    if(!hbridge) return FPGA_ERROR;

    if(osMutexAcquire(hbridge->mutex_id, timeout_ms) != osOK){
        return FPGA_BUSY;
    }

    status = FPGA_Read_Poll(hbridge, ADDR_S_DEBUG_MISC, &reg_val, timeout_ms);
    if(status == FPGA_OK){
        reg_val &= ~FPGA_MISC_BIST_EN_MASK;
        if(enable){
            reg_val |= FPGA_MISC_BIST_EN_INT;
        }
        status = FPGA_Write_Poll(hbridge, ADDR_S_DEBUG_MISC, reg_val, timeout_ms);
    }

    osMutexRelease(hbridge->mutex_id);
    return status;
}

/**
 * @brief  Configures dynamic multiplexers for diagnostic test points TP5..TP7.
 * @note   Target: Device ID 1, Reg 0x03 (5 bits per channel: TP5=[4:0], TP6=[9:5], TP7=[14:10]).
 */
FPGA_Status_t FPGA_Debug_Set_TP_Mux(FPGA_HandleTypeDef *hbridge, uint8_t tp5_sig, uint8_t tp6_sig, uint8_t tp7_sig, uint32_t timeout_ms){
    uint16_t reg_val = 0;

    reg_val |= (((uint16_t)tp5_sig << FPGA_TP_MUX_TP5_SHIFT) & FPGA_TP_MUX_TP5_MASK);
    reg_val |= (((uint16_t)tp6_sig << FPGA_TP_MUX_TP6_SHIFT) & FPGA_TP_MUX_TP6_MASK);
    reg_val |= (((uint16_t)tp7_sig << FPGA_TP_MUX_TP7_SHIFT) & FPGA_TP_MUX_TP7_MASK);

    return FPGA_Write_Poll(hbridge, ADDR_S_DEBUG_TP_MUX, reg_val, timeout_ms);
}
