/**
 ******************************************************************************
 * @file    version_control.c
 * @brief   Software and Hardware Version Information Implementation (PSPM).
 *          Compile-time string concatenation into Flash (.rodata).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "version_control.h"

/* Compile-time concatenation into a single Flash (.rodata) string */
static const char device_info_string[] =
    _VC_PRODUCT_NUMBER_
    _VC_BOARD_NUMBER_
    _VC_CHIP_DESIGNATION_
    _VC_SOFT_NAME_
    "Build:    " __DATE__ "  " __TIME__ "\n"
    _VC_SOFT_NOTE_
    _VC_SOFT_VERSION_;


const char* get_device_info(void) {
    return device_info_string;
}
