/**
 ******************************************************************************
 * @file    configuration.h
 * @brief   Non-Volatile System Configuration Manager for PSPM.
 *          Stores Passport, RDC PLL Settings, and ADC Peak/RMS Scale Mask.
 *          Target MCU: STM32F411 (Sector 4: 0x08010000, 64 KB).
 *          Board: PSPM (DIAS.901.02.00.300).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board_support_package.h"
#include "rdc.h"

/* ========================================================================= */
/*  FLASH MEMORY SETTINGS (Sector 4: 0x08010000 - 64 KB)                     */
/* ========================================================================= */

#define CFG_ADDRESS         0x08010000U  /* Base of internal Flash Sector 4 (64 KB) */
#define CFG_SECTOR          FLASH_SECTOR_4
#define CFG_START_ADDRESS   CFG_ADDRESS

/* NOTE: legacy "eeprom_*" driver names operate on this internal Flash region
   (memory-mapped read, unlock/erase/program write). No external EEPROM. */
#define CFG_PAGE_SIZE       1024U        /* Bytes programmed per page-write pass */
#define CFG_NUM_PAGE        1U           /* Image size = 1 KB (struct fits in one page) */
#define CFG_SIZE            (CFG_PAGE_SIZE * CFG_NUM_PAGE)

#define CLEAR_FLAG          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP   | \
                                                   FLASH_FLAG_OPERR | \
                                                   FLASH_FLAG_WRPERR| \
                                                   FLASH_FLAG_PGAERR| \
                                                   FLASH_FLAG_PGPERR| \
                                                   FLASH_FLAG_PGSERR)

/* ========================================================================= */
/*  METADATA & BOARD IDENTIFIERS                                             */
/* ========================================================================= */

#define CFG_HEADER              0x5053504DU /* Magic Signature: 'PSPM' */
#define CFG_BOARD_SERIAL_NUMBER 0ULL
#define CFG_STR                 "PSPM (DIAS.901.02.00.300) produced by Andrew Avramenko (2026)"

/* ========================================================================= */
/*  CONFIGURATION DATA STRUCTURES                                            */
/* ========================================================================= */

/* Main device parameters and passport structure */
typedef struct __attribute__((packed)) {
    /* 1. Passport & Identification */
    uint32_t        header;             /* Validation magic word (0x5053504D) */
    uint64_t        serial;             /* Board serial (factory placeholder 0; not bound to MCU UID) */
    char            str[256];           /* Board description & build passport string */

    /* 2. Resolver Tracking Loop Parameters (Type-II PLL) */
    rdc_math_cfg_t  rdc_cfg;            /* Carrier freq, loop bandwidth, damping ratio, max speed, add_PI */

    /* 3. Legacy byte (was AC Peak/RMS mask, scaling is fixed Peak now).
          Kept in place so the Flash image layout stays backward-compatible. */
    uint8_t         reserved0;

    /* 4. Reserved storage for future calibrations */
    uint8_t         reserved[255];

} t_dev_cfg;

/* Memory image union (1024 bytes total, checksum at word index 511) */
typedef union {
    t_dev_cfg   item;
    uint8_t     u8[CFG_SIZE];
    uint16_t    u16[CFG_SIZE / 2];
} t_memory_cfg;

/* ========================================================================= */
/*  EXPORTED VARIABLES & APIS                                                */
/* ========================================================================= */

extern t_memory_cfg dev_cfg;

bool	get_dev_cfg(void);     /* Boot load+validate; formats factory image on corruption */
bool	get_dev_status(void);  /* One-shot boot result of get_dev_cfg() */
bool	save_setting(void);    /* Validate live PLL cfg -> rewrite Sector 4 -> RDC_Init */

#ifdef __cplusplus
}
#endif

#endif /* CONFIGURATION_H */
