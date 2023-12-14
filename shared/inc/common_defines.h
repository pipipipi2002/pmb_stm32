#ifndef INC_PMB_COMMON_DEFINES_H
#define INC_PMB_COMMON_DEFINES_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define BOOTLOADER_SIZE         (0x8000U)        // 32KiBi 
#define MAIN_APP_START_ADDR     (FLASH_BASE + BOOTLOADER_SIZE)

#endif