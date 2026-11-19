/**
 ******************************************************************************
 * @file    version_control.h
 * @brief   Software and Hardware Version Information for PSPM Board.
 *          Target MCU: STM32F411RET6TR (DD1) on DIAS.901.02.00.300.
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef VERSION_CONTROL_H_
#define VERSION_CONTROL_H_

#include "board_support_package.h"

/* ========================================================================= */
/*  STATIC STRING DEFINITIONS (PSPM BOARD: DIAS.901.02.00.300)               */
/* ========================================================================= */

#define _VC_PRODUCT_NUMBER_     "Primary Signal Processing Module (PSPM)\n"
#define _VC_BOARD_NUMBER_       "Board:    DIAS.901.02.00.300\n"
#define _VC_CHIP_DESIGNATION_   "Chip:     DD1 - STM32F411RET6TR\n"
#define _VC_SOFT_NAME_          "Software: PSPM Combat Firmware\n"
#define _VC_SOFT_NOTE_          "SW:       produced by mr. Andrew Avramenko\n"
#define _VC_SOFT_VERSION_       "Version:  2.0\n"

/* ========================================================================= */
/*  PUBLIC API                                                               */
/* ========================================================================= */

/**
 * @brief  Returns pointer to static device info string stored in Flash (.rodata).
 *         Zero RAM overhead.
 */
const char* get_device_info(void);

#endif /* VERSION_CONTROL_H_ */
