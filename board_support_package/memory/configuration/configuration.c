/**
 ******************************************************************************
 * @file    configuration.c
 * @brief   Non-Volatile System Configuration Implementation (Sector 4).
 *          Stores Passport, RDC Settings, and ADC Peak/RMS Configuration.
 *          Board: PSPM (DIAS.901.02.00.300).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "configuration.h"
#include <string.h>

/* Global configuration instance (RAM image of Flash Sector 4) */
t_memory_cfg dev_cfg;
static bool  dev_cfg_ready = false;

/* Private function prototypes */
static uint16_t	calc_sum(uint16_t *data, uint32_t len);
static bool		__eeprom_memory_write(uint8_t *dst, uint32_t addr);
static void		eeprom_memory_read(uint8_t *dst, const uint8_t *src, uint32_t len);
static bool		eeprom_memory_erase(uint32_t sector);
static bool		eeprom_memory_write_page(uint8_t *dst, uint32_t addr);

/* ========================================================================= */
/*  PUBLIC API IMPLEMENTATION                                                */
/* ========================================================================= */

/*
 * Load the config image from Flash Sector 4 and validate it (magic header +
 * 16-bit tail checksum, up to 5 retries with 5 ms delay). Unprogrammed or
 * corrupted storage is formatted in place with factory defaults. On both
 * paths the RDC PLL parameters are published into the live g_rdc_math_cfg.
 * Call once at boot, before RDC_Init().
 */
bool get_dev_cfg(void) {
	bool result = true;
	uint32_t i = 0;

	do {
		eeprom_memory_read(dev_cfg.u8, (uint8_t*)CFG_START_ADDRESS, CFG_SIZE);

		/* Validate magic header and 16-bit tail checksum */
		result = (dev_cfg.item.header == CFG_HEADER);
		result &= (dev_cfg.u16[CFG_SIZE / 2 - 1] == calc_sum(dev_cfg.u16, CFG_SIZE / 2 - 1));

		i++;
		HAL_Delay(5);

	} while ((i < 5) && (result == false));

	/* If Flash is unprogrammed or corrupted, format with safe factory defaults */
	if (!result) {
		memset(&dev_cfg, 0, sizeof(t_memory_cfg));

		/* 1. Factory Passport */
		dev_cfg.item.header = CFG_HEADER;
		dev_cfg.item.serial = CFG_BOARD_SERIAL_NUMBER;
		strncpy(dev_cfg.item.str, CFG_STR, sizeof(dev_cfg.item.str) - 1);
		dev_cfg.item.str[sizeof(dev_cfg.item.str) - 1] = '\0';

		/* 2. RDC Default Mathematical Configuration */
		dev_cfg.item.rdc_cfg.carrier_freq_hz   = RDC_DEFAULT_CARRIER_FREQ_HZ;   // 400.0 Hz
		dev_cfg.item.rdc_cfg.loop_bandwidth_hz = RDC_DEFAULT_LOOP_BANDWIDTH_HZ; // 15.0 Hz
		dev_cfg.item.rdc_cfg.damping_ratio     = RDC_DEFAULT_DAMPING_RATIO;     // 0.707 (Butterworth)
		dev_cfg.item.rdc_cfg.max_speed_dps     = RDC_DEFAULT_MAX_SPEED_DPS;     // 1000.0 deg/s
		dev_cfg.item.rdc_cfg.add_PI            = RDC_DEFAULT_ADD_PI;            // true (+180 deg)

		/* 3. Legacy reserved0 byte stays 0 (memset above) */

		/* Apply defaults to active tracking loop */
		g_rdc_math_cfg = dev_cfg.item.rdc_cfg;

		/* 4. Calculate Checksum and write factory image to Flash Sector 4 */
		dev_cfg.u16[CFG_SIZE / 2 - 1] = calc_sum(dev_cfg.u16, CFG_SIZE / 2 - 1);

		result = eeprom_memory_erase(CFG_SECTOR);
		result &= __eeprom_memory_write(dev_cfg.u8, CFG_START_ADDRESS);
	} else {
		/* Load validated stored parameters into active tracking filter */
		g_rdc_math_cfg = dev_cfg.item.rdc_cfg;
	}

	dev_cfg_ready = result;
	return result;
}

/* One-shot boot result: true = image validated or freshly formatted. */
bool get_dev_status(void) {
	return dev_cfg_ready;
}

/*
 * Commit the live g_rdc_math_cfg to non-volatile storage: invalid values are
 * rejected BEFORE any flash access (caller must roll back on false), then the
 * config is snapshotted into dev_cfg, checksum refreshed, Sector 4 rewritten
 * and the PLL re-initialized with the new gains.
 * WARNING: flash erase/program run with IRQs disabled and fetch stalled for
 * ~0.5-1 s: 400 Hz loop and UART RX freeze. Bench use only, never mid-session.
 */
bool save_setting(void) {
	bool result = true;

	/* Validate parameters before writing to Flash */
	if (!RDC_ValidateConfig(&g_rdc_math_cfg)) {
		return false;
	}

	dev_cfg.item.rdc_cfg = g_rdc_math_cfg;

	/* Recalculate checksum before writing */
	dev_cfg.u16[CFG_SIZE / 2 - 1] = calc_sum(dev_cfg.u16, CFG_SIZE / 2 - 1);

	result = eeprom_memory_erase(CFG_SECTOR);
	result &= __eeprom_memory_write(dev_cfg.u8, CFG_START_ADDRESS);

	/* Re-initialize active tracking loop with updated parameters */
	if (result) {
		RDC_Init(&g_rdc_math_cfg);
	}

	return result;
}

/* ========================================================================= */
/*  PRIVATE FLASH LOW-LEVEL DRIVER (legacy "eeprom" prefix = internal flash) */
/* ========================================================================= */

/* Internal flash is memory-mapped: a read is a plain memcpy. */
static void eeprom_memory_read(uint8_t *dst, const uint8_t *src, uint32_t len) {
	memcpy(dst, src, len);
}

/* Erase one sector, IRQs off; CPU is fetch-stalled while the operation runs. */
static bool eeprom_memory_erase(uint32_t sector) {
	bool result = true;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError = 0;

	__disable_irq();

	result &= (HAL_FLASH_Unlock() == HAL_OK);
	if (!result) goto finish;

	CLEAR_FLAG;

	EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector       = sector;
	EraseInitStruct.NbSectors    = 1;

	result &= (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK);

finish:
	result &= (HAL_FLASH_Lock() == HAL_OK);
	__enable_irq();

	return result;
}

/* Program one 1 KB page byte-wise, IRQs off; aborts on first failed byte. */
static bool eeprom_memory_write_page(uint8_t *dst, uint32_t addr) {
	bool result = true;
	uint8_t *pdata = dst;

	__disable_irq();

	result &= (HAL_FLASH_Unlock() == HAL_OK);
	if (!result) goto finish;

	for (uint32_t i = 0; i < CFG_PAGE_SIZE; i++) {
		CLEAR_FLAG;
		result &= (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, *pdata) == HAL_OK);
		addr += 1;
		pdata++;
		if (!result) goto finish;
	}

finish:
	result &= (HAL_FLASH_Lock() == HAL_OK);
	__enable_irq();

	return result;
}

/* Write the full image page by page (CFG_NUM_PAGE sequential pages). */
static bool __eeprom_memory_write(uint8_t *dst, uint32_t addr) {
	uint32_t address = addr;
	bool result = true;

	for (uint32_t i = 0; i < CFG_NUM_PAGE; i++) {
		result &= eeprom_memory_write_page(dst, address);
		dst     += CFG_PAGE_SIZE;
		address += CFG_PAGE_SIZE;
	}
	return result;
}

/* Internet-style checksum: one's complement of the folded 16-bit word sum. */
static uint16_t calc_sum(uint16_t *data, uint32_t len) {
	uint32_t cs = 0;
	uint16_t *dataptr = data;

	for (uint32_t i = 0; i < len; i++) {
		cs += *dataptr++;
	}

	uint32_t carry;
	do {
		carry = cs >> 16;
		cs    = (cs & 0xFFFF) + carry;
	} while (carry != 0);

	return ~cs;
}
