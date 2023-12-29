#ifndef INC_PMB_COMMON_DEFINES_H
#define INC_PMB_COMMON_DEFINES_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <libopencm3/stm32/memorymap.h>

#define BOOTLOADER_SIZE         (0x8000U)        // 32KiBi 
#define BOOTLOADER_START_ADDR   (FLASH_BASE)
#define MAIN_APP_START_ADDR     (BOOTLOADER_START_ADDR + BOOTLOADER_SIZE)
#define MAIN_APP_SIZE_MAX       ((1024U * 128U) - BOOTLOADER_SIZE)

#endif