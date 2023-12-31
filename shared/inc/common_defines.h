#ifndef INC_PMB_COMMON_DEFINES_H
#define INC_PMB_COMMON_DEFINES_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <libopencm3/stm32/memorymap.h>

#define BOOTLOADER_START_ADDR   (FLASH_BASE)
#define BOOTLOADER_SIZE         (0x7800U)        // 30 KiBi
#define AMD_START_ADDR          (BOOTLOADER_START_ADDR + BOOTLOADER_SIZE)
#define AMD_SIZE                (0x800U)         // 2 KiB
#define MAIN_APP_START_ADDR     (AMD_START_ADDR + AMD_SIZE)
#define MAIN_APP_SIZE_MAX       ((1024U * 128U) - (BOOTLOADER_SIZE + AMD_SIZE))

#define BOOTLOADER_PAGE_START   (0)
#define BOOTLOADER_PAGE_END     (14)
#define APP_METADATA_PAGE_START (15)
#define APP_METADATA_PAGE_END   (15)
#define MAIN_APP_PAGE_START     (16)
#define MAIN_APP_PAGE_END       (63)
#define MEMORY_PAGE_SIZE        (0x800)

#endif