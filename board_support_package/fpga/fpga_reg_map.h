/**
  ******************************************************************************
  * @file    fpga_reg_map.h
  * @brief   FPGA Register Map & DMA Packet Definitions for PSPM System.
  *          10-bit Address Mapping: [9:6] Module ID (4 bits), [5:0] Reg Offset (6 bits).
  *          Target: Xilinx Spartan-6 + STM32F4 (SPI Mode 00).
  *          All comments in ASCII English.
  ******************************************************************************
  */

#ifndef FPGA_REG_MAP_H
#define FPGA_REG_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================= */
/*  1. BUS ARCHITECTURE & ADDRESSING MACROS                                  */
/* ========================================================================= */
#define FPGA_S_ADDR_WIDTH            10U   // Total address bus width (bits 9..0)
#define FPGA_S_DEV_ADDR_WIDTH        4U    // Module Device ID width (bits 9..6)
#define FPGA_S_CHIP_ADDR_WIDTH       6U    // Register Offset width (bits 5..0)

/**
 * @brief  Helper Macro to Calculate 10-bit SPI Address
 * @param  dev: Device ID (1..6, 4 bits)
 * @param  reg: Register Offset inside device (0..63, 6 bits)
 * @return 10-bit hardware address
 */
#define FPGA_S_ADDR(dev, reg)        ((((uint16_t)(dev) & 0x0FU) << FPGA_S_CHIP_ADDR_WIDTH) | \
                                       ((uint16_t)(reg) & 0x3FU))

/* DMA Packet Sizing */
#define FPGA_DMA_READ_WORDS          34U   // Telemetry burst size (544 bits = 34 words, incl. STATUS_WORD)
#define FPGA_DMA_WRITE_WORDS         10U   // DAC command burst size (160 bits = 10 words)

/* DMA Command Headers (Word 0 in DMA Stream) */
#define FPGA_DMA_CMD_READ            0x2000U // Bit 13 = Read Enable
#define FPGA_DMA_CMD_WRITE           0x800AU // Bit 15 = Write, Length = 10 (0x0A)
#define FPGA_DMA_CMD_RW              0xC00AU // Bit 15 = Write, Bit 14 = Read Enable (MISO active), Length = 10

/* FPGA Sticky Status Word (Burst Word 33, cleared by next DMA snapshot) */
#define FPGA_ST_ADC_ERR              0x0001U // Bit 0: DA21/DA22/DA45 sample watchdog timeout
#define FPGA_ST_RES_ERR              0x0002U // Bit 1: Resolver ADC sample watchdog timeout
#define FPGA_ST_RES_OVR              0x0004U // Bit 2: Resolver overrun (data not consumed)
#define FPGA_ST_ERR_12               0x0008U // Bit 3: Comparator 1-2 error (no UON period)
#define FPGA_ST_ERR_23               0x0010U // Bit 4: Comparator 2-3 error (no UON period)
#define FPGA_ST_ALL_MASK             0x001FU // All defined sticky flags

/* ========================================================================= */
/*  2. MODULE 1: DEBUG & SYSTEM CONFIGURATION (ID = 1)                       */
/* ========================================================================= */
#define FPGA_S_DEV_DEBUG_ID          1U

#define REG_S_DEBUG_FEEDBACK_OFF     0x00U // [RW] Scratchpad echo register
#define REG_S_DEBUG_CONST_OFF        0x01U // [RO] Alive check validation constant (0xDEAD)
#define REG_S_DEBUG_MISC_OFF         0x02U // [RW] System miscellaneous control register
#define REG_S_DEBUG_TP_MUX_OFF       0x03U // [RW] Dynamic Testpoint Multiplexer routing
// Offset 0x04 in RTL is a [WO] IPROG reconfiguration trigger. The design uses a
// single FPGA image, so no firmware API for it exists by intent.

#define ADDR_S_DEBUG_FEEDBACK        FPGA_S_ADDR(FPGA_S_DEV_DEBUG_ID, REG_S_DEBUG_FEEDBACK_OFF)
#define ADDR_S_DEBUG_CONST           FPGA_S_ADDR(FPGA_S_DEV_DEBUG_ID, REG_S_DEBUG_CONST_OFF)
#define ADDR_S_DEBUG_MISC            FPGA_S_ADDR(FPGA_S_DEV_DEBUG_ID, REG_S_DEBUG_MISC_OFF)
#define ADDR_S_DEBUG_TP_MUX          FPGA_S_ADDR(FPGA_S_DEV_DEBUG_ID, REG_S_DEBUG_TP_MUX_OFF)

#define FPGA_DEBUG_CONST_VAL         0xDEADU // Validation constant from Verilog
#define FPGA_DEBUG_ECHO_TEST_VAL     0x55AAU // SPI Bus Integrity Checkerboard Test Pattern

/**
 * REG_S_DEBUG_MISC (0x02) Bit Definitions:
 *
 * Bits [2:0]  : LED Control (1 = LED On, 0 = LED Off)
 *               Bit 0: VD35 (Green)
 *               Bit 1: VD36 (Yellow)
 *               Bit 2: VD37 (Red)
 *
 * Bit 3       : RS-485 / UART Multiplexer Channel Select (from STM32 USART2)
 *               0 = Channel 1 (Transceiver DD4: tx2, rx2, de2)
 *               1 = Channel 2 (Transceiver DD5: tx3, rx3, de3)
 *
 * Bit 4       : Resolver Reference Phase Hardware Multiplexer
 *               0 = Synchronize Resolver to Phase 1-2 (UON12, 0 deg)
 *               1 = Synchronize Resolver to Phase 2-3 (UON23, 120 deg)
 *
 * Bit 5       : Internal 400 Hz 3-Phase BIST Generator Enable
 *               0 = Use external physical comparator inputs (UON12, UON23)
 *               1 = Inject internal mathematical 400 Hz BIST waveforms
 *
 * Bits [15:6] : Reserved
 */
#define FPGA_MISC_LED_GREEN          0x0001U // Bit 0: VD35 (Green)
#define FPGA_MISC_LED_YELLOW         0x0002U // Bit 1: VD36 (Yellow)
#define FPGA_MISC_LED_RED            0x0004U // Bit 2: VD37 (Red)
#define FPGA_MISC_LED_MASK           (FPGA_MISC_LED_GREEN | FPGA_MISC_LED_YELLOW | FPGA_MISC_LED_RED)

#define FPGA_MISC_UART_MUX_DD4       0x0000U // Bit 3 = 0: Route USART2 to DD4
#define FPGA_MISC_UART_MUX_DD5       0x0008U // Bit 3 = 1: Route USART2 to DD5
#define FPGA_MISC_UART_MUX_MASK      0x0008U

#define FPGA_MISC_RES_PHASE_U12      0x0000U // Bit 4 = 0: Resolver synced to UON12
#define FPGA_MISC_RES_PHASE_U23      0x0010U // Bit 4 = 1: Resolver synced to UON23
#define FPGA_MISC_RES_PHASE_MASK     0x0010U

#define FPGA_MISC_BIST_EN_EXT        0x0000U // Bit 5 = 0: Physical pins
#define FPGA_MISC_BIST_EN_INT        0x0020U // Bit 5 = 1: BIST Generator active
#define FPGA_MISC_BIST_EN_MASK       0x0020U

/**
 * REG_S_DEBUG_TP_MUX (0x03) Bit Definitions (5 bits per Testpoint):
 *
 * Bits [4:0]   : Signal selector for TP5 (Physical Pin 81)
 * Bits [9:5]   : Signal selector for TP6 (Physical Pin 80)
 * Bits [14:10] : Signal selector for TP7 (Physical Pin 79)
 * Bit  15      : Reserved
 */
#define FPGA_TP_MUX_TP5_SHIFT        0U
#define FPGA_TP_MUX_TP5_MASK         (0x1FU << FPGA_TP_MUX_TP5_SHIFT)
#define FPGA_TP_MUX_TP6_SHIFT        5U
#define FPGA_TP_MUX_TP6_MASK         (0x1FU << FPGA_TP_MUX_TP6_SHIFT)
#define FPGA_TP_MUX_TP7_SHIFT        10U
#define FPGA_TP_MUX_TP7_MASK         (0x1FU << FPGA_TP_MUX_TP7_SHIFT)

/* ========================================================================= */
/*  3. MODULE 2: INTERRUPT CONTROLLER (ID = 2)                               */
/* ========================================================================= */
#define FPGA_S_DEV_INT_CTRL_ID       2U

#define REG_S_INT_PENDING_OFF        0x00U // [R] Masked Pending / [W1C] Clear Pending
#define REG_S_INT_MASK_OFF           0x01U // [RW] Mask (1 = Enabled, 0 = Disabled)
#define REG_S_INT_EDGE_SEL_OFF       0x02U // [RW] Edge Select (0 = Rising, 1 = Falling)
#define REG_S_INT_CTRL_OFF           0x03U // [RW] Global Control (Bit 0 = Global Enable)

#define ADDR_S_INT_PENDING           FPGA_S_ADDR(FPGA_S_DEV_INT_CTRL_ID, REG_S_INT_PENDING_OFF)
#define ADDR_S_INT_MASK              FPGA_S_ADDR(FPGA_S_DEV_INT_CTRL_ID, REG_S_INT_MASK_OFF)
#define ADDR_S_INT_EDGE_SEL          FPGA_S_ADDR(FPGA_S_DEV_INT_CTRL_ID, REG_S_INT_EDGE_SEL_OFF)
#define ADDR_S_INT_CTRL              FPGA_S_ADDR(FPGA_S_DEV_INT_CTRL_ID, REG_S_INT_CTRL_OFF)

#define FPGA_IRQ_LINE_ADC12_READY    0x0001U // Bit 0: 400 Hz Data Ready
#define FPGA_IRQ_LINE_FREQ_ERR       0x0002U // Bit 1: Frequency Watchdog Error
#define FPGA_IRQ_LINES_ALL_MASK      (FPGA_IRQ_LINE_ADC12_READY | FPGA_IRQ_LINE_FREQ_ERR)

#define FPGA_INT_CTRL_GIE_ENABLE     0x0001U // Bit 0: Global Interrupt Enable

/* ========================================================================= */
/*  4. MODULE 3: 10-CHANNEL DAC SUBSYSTEM - AD5543 (ID = 3)                  */
/* ========================================================================= */
#define FPGA_S_DEV_DAC_ID            3U

#define REG_S_DAC1_DATA_OFF          0x01U
#define REG_S_DAC2_DATA_OFF          0x02U
#define REG_S_DAC3_DATA_OFF          0x03U
#define REG_S_DAC4_DATA_OFF          0x04U
#define REG_S_DAC5_DATA_OFF          0x05U
#define REG_S_DAC6_DATA_OFF          0x06U
#define REG_S_DAC7_DATA_OFF          0x07U
#define REG_S_DAC8_DATA_OFF          0x08U
#define REG_S_DAC9_DATA_OFF          0x09U
#define REG_S_DAC10_DATA_OFF         0x0AU

#define REG_S_DAC_BIPOLAR_CFG_OFF    0x0BU // [RW] Bipolar Mode Mask
#define REG_S_DAC_STATUS_OFF         0x0CU // [RO] Status
#define REG_S_DAC_HARD_CTRL_OFF      0x0DU // [RW] Hardware DMA Mode Control

#define ADDR_S_DAC_DATA(ch)          FPGA_S_ADDR(FPGA_S_DEV_DAC_ID, (ch))
#define ADDR_S_DAC_BIPOLAR_CFG       FPGA_S_ADDR(FPGA_S_DEV_DAC_ID, REG_S_DAC_BIPOLAR_CFG_OFF)
#define ADDR_S_DAC_STATUS            FPGA_S_ADDR(FPGA_S_DEV_DAC_ID, REG_S_DAC_STATUS_OFF)
#define ADDR_S_DAC_HARD_CTRL         FPGA_S_ADDR(FPGA_S_DEV_DAC_ID, REG_S_DAC_HARD_CTRL_OFF)

/* ========================================================================= */
/*  5. MODULE 4: RESOLVER / RDC SUBSYSTEM (ID = 4)                           */
/* ========================================================================= */
#define FPGA_S_DEV_RESOLVER_ID       4U

#define REG_S_RES_CTRL_OFF           0x00U // [RW] B15: ACK, B14: IRQ_EN, B[7:0]: SEL_MATRIX
#define REG_S_RES_STATUS_OFF         0x01U // [RO] B2: BUSY, B1: OVERRUN_ERR, B0: DATA_VALID
#define REG_S_RES_SAMPLES_CNT_OFF    0x02U // [RO] Number of samples per 400Hz period (~250)
#define REG_S_RES_SIN_ROLL_L_OFF     0x03U // [RO] 32-bit Sin Roll (Low Word)
#define REG_S_RES_SIN_ROLL_H_OFF     0x04U // [RO] 32-bit Sin Roll (High Word)
#define REG_S_RES_COS_ROLL_L_OFF     0x05U // [RO] 32-bit Cos Roll (Low Word)
#define REG_S_RES_COS_ROLL_H_OFF     0x06U // [RO] 32-bit Cos Roll (High Word)
#define REG_S_RES_SIN_PITCH_L_OFF    0x07U // [RO] 32-bit Sin Pitch (Low Word)
#define REG_S_RES_SIN_PITCH_H_OFF    0x08U // [RO] 32-bit Sin Pitch (High Word)
#define REG_S_RES_COS_PITCH_L_OFF    0x09U // [RO] 32-bit Cos Pitch (Low Word)
#define REG_S_RES_COS_PITCH_H_OFF    0x0AU // [RO] 32-bit Cos Pitch (High Word)

#define ADDR_S_RES_CTRL              FPGA_S_ADDR(FPGA_S_DEV_RESOLVER_ID, REG_S_RES_CTRL_OFF)
#define ADDR_S_RES_STATUS            FPGA_S_ADDR(FPGA_S_DEV_RESOLVER_ID, REG_S_RES_STATUS_OFF)
#define ADDR_S_RES_SAMPLES_CNT       FPGA_S_ADDR(FPGA_S_DEV_RESOLVER_ID, REG_S_RES_SAMPLES_CNT_OFF)

/* ========================================================================= */
/*  6. MODULE 5: 3-PHASE COMPARATORS & FREQUENCY WATCHDOG (ID = 5)           */
/* ========================================================================= */
#define FPGA_S_DEV_COMP_ID           5U

#define REG_S_COMP_CTRL_OFF          0x00U // [RW] B15: ACK, B14: IRQ_EN, B0: EN_MEASUREMENT
#define FPGA_COMP_CTRL_IRQ_EN        0x4000U // Enable freq-error IRQ line to interrupt controller
#define FPGA_COMP_CTRL_EN_MEAS       0x0001U // Measurement engine enable (reset default = 1)
#define REG_S_COMP_STATUS_OFF        0x01U // [RO] B3: ERR_23, B2: ERR_12, B1: STATE_23, B0: STATE_12
#define REG_S_COMP_VERIFY_OFF        0x02U // [RW] Fast Verification time (1 LSB = 10ns)
#define REG_S_COMP_BLANKING_OFF      0x03U // [RW] Blanking Lockout time (1 LSB = 10ns)
#define REG_S_COMP_MIN_PER_OFF       0x04U // [RW] Min Period / 425Hz (1 LSB = 0.1us, Def: 23529)
#define REG_S_COMP_MAX_PER_OFF       0x05U // [RW] Max Period / 375Hz (1 LSB = 0.1us, Def: 26666)
#define REG_S_COMP_PER_12_OFF        0x06U // [RO] Measured Period Phase 1-2 (1 LSB = 0.1us)
#define REG_S_COMP_PER_23_OFF        0x07U // [RO] Measured Period Phase 2-3 (1 LSB = 0.1us)

#define ADDR_S_COMP_CTRL             FPGA_S_ADDR(FPGA_S_DEV_COMP_ID, REG_S_COMP_CTRL_OFF)
#define ADDR_S_COMP_STATUS           FPGA_S_ADDR(FPGA_S_DEV_COMP_ID, REG_S_COMP_STATUS_OFF)
#define ADDR_S_COMP_VERIFY           FPGA_S_ADDR(FPGA_S_DEV_COMP_ID, REG_S_COMP_VERIFY_OFF)
#define ADDR_S_COMP_BLANKING         FPGA_S_ADDR(FPGA_S_DEV_COMP_ID, REG_S_COMP_BLANKING_OFF)
#define ADDR_S_COMP_MIN_PER          FPGA_S_ADDR(FPGA_S_DEV_COMP_ID, REG_S_COMP_MIN_PER_OFF)
#define ADDR_S_COMP_MAX_PER          FPGA_S_ADDR(FPGA_S_DEV_COMP_ID, REG_S_COMP_MAX_PER_OFF)
#define ADDR_S_COMP_PER_12           FPGA_S_ADDR(FPGA_S_DEV_COMP_ID, REG_S_COMP_PER_12_OFF)
#define ADDR_S_COMP_PER_23           FPGA_S_ADDR(FPGA_S_DEV_COMP_ID, REG_S_COMP_PER_23_OFF)

#define FPGA_COMP_DEF_MIN_PER_TICKS  23529U  // 2.3529 ms (425.0 Hz)
#define FPGA_COMP_DEF_MAX_PER_TICKS  26666U  // 2.6666 ms (375.0 Hz)

/* ========================================================================= */
/*  7. MODULE 6: ADC SUBSYSTEM - DA21 4AC, DA22 4AC, DA45 4DC (ID = 6)       */
/* ========================================================================= */
#define FPGA_S_DEV_ADC_BLOCK_ID      6U

#define REG_S_ADC_CTRL_OFF           0x00U // [RW] B15: ACK, B14: IRQ_EN (data ready, UON23)
#define FPGA_ADC_CTRL_IRQ_EN         0x4000U // Enable 400Hz data-ready IRQ line to interrupt controller
#define REG_S_ADC_STATUS_OFF         0x01U // [RO] B2: VALID_DA45, B1: VALID_DA22, B0: VALID_DA21

/* DA21: Elevation 4x AC (Synced to UON23) */
#define REG_S_DA21_CNT_OFF           0x02U // [RO] 16-bit AC Sample count
#define REG_S_DA21_AC1_L_OFF         0x03U // [RO] 32-bit AC Channel 1 (Low Word)
#define REG_S_DA21_AC1_H_OFF         0x04U // [RO] 32-bit AC Channel 1 (High Word)
#define REG_S_DA21_AC2_L_OFF         0x05U // [RO] 32-bit AC Channel 2 (Low Word)
#define REG_S_DA21_AC2_H_OFF         0x06U // [RO] 32-bit AC Channel 2 (High Word)
#define REG_S_DA21_AC3_L_OFF         0x07U // [RO] 32-bit AC Channel 3 (Low Word)
#define REG_S_DA21_AC3_H_OFF         0x08U // [RO] 32-bit AC Channel 3 (High Word)
#define REG_S_DA21_AC4_L_OFF         0x09U // [RO] 32-bit AC Channel 4 (Low Word)
#define REG_S_DA21_AC4_H_OFF         0x0AU // [RO] 32-bit AC Channel 4 (High Word)

/* DA22: Azimuth 4x AC (Synced to UON12) */
#define REG_S_DA22_CNT_OFF           0x0BU // [RO] 16-bit AC Sample count
#define REG_S_DA22_AC1_L_OFF         0x0CU // [RO] 32-bit AC Channel 1 (Low Word)
#define REG_S_DA22_AC1_H_OFF         0x0DU // [RO] 32-bit AC Channel 1 (High Word)
#define REG_S_DA22_AC2_L_OFF         0x0EU // [RO] 32-bit AC Channel 2 (Low Word)
#define REG_S_DA22_AC2_H_OFF         0x0FU // [RO] 32-bit AC Channel 2 (High Word)
#define REG_S_DA22_AC3_L_OFF         0x10U // [RO] 32-bit AC Channel 3 (Low Word)
#define REG_S_DA22_AC3_H_OFF         0x11U // [RO] 32-bit AC Channel 3 (High Word)
#define REG_S_DA22_AC4_L_OFF         0x12U // [RO] 32-bit AC Channel 4 (Low Word)
#define REG_S_DA22_AC4_H_OFF         0x13U // [RO] 32-bit AC Channel 4 (High Word)

/* DA45: Telemetry 4x DC Channels */
#define REG_S_DA45_DC1_OFF           0x14U // [RO] 16-bit DC Channel 1
#define REG_S_DA45_DC2_OFF           0x15U // [RO] 16-bit DC Channel 2
#define REG_S_DA45_DC3_OFF           0x16U // [RO] 16-bit DC Channel 3
#define REG_S_DA45_DC4_OFF           0x17U // [RO] 16-bit DC Channel 4
#define REG_S_ADC_DECIMATION_OFF     0x18U // [RW] Decimation Shift Control

#define ADDR_S_ADC_CTRL              FPGA_S_ADDR(FPGA_S_DEV_ADC_BLOCK_ID, REG_S_ADC_CTRL_OFF)
#define ADDR_S_ADC_STATUS            FPGA_S_ADDR(FPGA_S_DEV_ADC_BLOCK_ID, REG_S_ADC_STATUS_OFF)
#define ADDR_S_ADC_DECIMATION        FPGA_S_ADDR(FPGA_S_DEV_ADC_BLOCK_ID, REG_S_ADC_DECIMATION_OFF)

/* ========================================================================= */
/*  8. DMA STREAMING PACKET STRUCTURES (DATA PLANE: 34 WORDS = 68 BYTES)     */
/* ========================================================================= */

/**
 * @brief  Complete Telemetry DMA Burst Packet read from FPGA (34 words = 68 bytes)
 * @note   Packed with 1-byte alignment. Maps Little-Endian words directly to variables.
 */
typedef struct __attribute__((packed)) {
    /* --- 1. Resolver Subsystem DA36 (Words 0..8 / 144 bits) --- */
    int32_t  res_sin_roll;       /* Words 0..1  : Demodulated Integrated Sin Roll */
    int32_t  res_cos_roll;       /* Words 2..3  : Demodulated Integrated Cos Roll */
    int32_t  res_sin_pitch;      /* Words 4..5  : Demodulated Integrated Sin Pitch */
    int32_t  res_cos_pitch;      /* Words 6..7  : Demodulated Integrated Cos Pitch */
    uint16_t res_samples_cnt;    /* Word  8     : Total samples accumulated (~250) */

    /* --- 2. DA45: 4x DC Telemetry Channels (Words 9..12 / 64 bits) ---
       AD7367 A/B interleave: ch1=A1, ch2=B1, ch3=A2, ch4=B2 (FPGA is truth) */
    int16_t  da45_dc1;           /* Word  9     : SPEED_TANK (A1) */
    int16_t  da45_dc2;           /* Word  10    : COS_FI (B1) */
    int16_t  da45_dc3;           /* Word  11    : SPEED_WIND (A2) */
    int16_t  da45_dc4;           /* Word  12    : RESERVE (B2) */

    /* --- 3. DA22: Azimuth Subsystem 4x AC (Words 13..21 / 144 bits, Synced to UON12) --- */
    uint16_t da22_ac_cnt;        /* Word  13    : Sample count (~250) */
    int32_t  da22_ac1;           /* Words 14..15: ERR_AZ (A1) */
    int32_t  da22_ac2;           /* Words 16..17: ERR_AZ1O (B1) */
    int32_t  da22_ac3;           /* Words 18..19: RESERVE / VON12 monitor (A2) */
    int32_t  da22_ac4;           /* Words 20..21: W_AZ rate input (B2) */

    /* --- 4. DA21: Elevation Subsystem 4x AC (Words 22..30 / 144 bits, Synced to UON23) --- */
    uint16_t da21_ac_cnt;        /* Word  22    : Sample count (~250) */
    int32_t  da21_ac1;           /* Words 23..24: ERR_EL (A1) */
    int32_t  da21_ac2;           /* Words 25..26: ERR_EL1O (B1) */
    int32_t  da21_ac3;           /* Words 27..28: W_EL rate input (A2) */
    int32_t  da21_ac4;           /* Words 29..30: GUN_POS transmitter amplitude (B2) */

    /* --- 5. 3-Phase Network Frequency Watchdog (Words 31..32 / 32 bits) --- */
    uint16_t period_uon12;       /* Word  31    : Phase 1-2 Period (1 LSB = 0.1 us) */
    uint16_t period_uon23;       /* Word  32    : Phase 2-3 Period (1 LSB = 0.1 us) */

    /* --- 6. Sticky Error Flags (Word 33 / 16 bits) --- */
    uint16_t status_word;        /* Word  33    : FPGA_ST_* sticky flags (cleared by snapshot) */
} FpgaDmaTelemetryPacket_t;

/**
 * @brief  10-Channel DAC Command Packet sent to FPGA over DMA (10 words = 20 bytes)
 */
typedef struct __attribute__((packed)) {
    int16_t dac_val[10];         /* Target values for 10x AD5543 DACs in int16_t format */
} FpgaDmaDacPacket_t;

/**
 * @brief  Raw DMA SPI Receive Buffer (35 Half-Words = 70 Bytes)
 */
typedef struct __attribute__((packed)) {
    uint16_t                 cmd_dummy_word; /* Discarded Word 0 during Command phase */
    FpgaDmaTelemetryPacket_t telemetry;      /* 34 Words of valid Telemetry */
} FpgaDmaRxFrame_t;

/**
 * @brief  Raw DMA SPI Transmit Buffer (Includes 1-word Command Header)
 */
typedef struct __attribute__((packed)) {
    uint16_t                 cmd_header;     /* Word 0: 0x800A (Write 10 DACs) */
    FpgaDmaDacPacket_t       dac_payload;    /* 10 Words of DAC values */
} FpgaDmaTxFrame_t;

#ifdef __cplusplus
}
#endif

#endif /* FPGA_REG_MAP_H */
