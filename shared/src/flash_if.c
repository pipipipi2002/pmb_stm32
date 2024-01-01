#include "flash_if.h"
#include <libopencm3/stm32/flash.h>
#include "metadata.h"

/**
 * @brief Erase Main Application Space
 * 
 */
void flashif_eraseMainApplication() {
    flash_unlock();
    for (uint8_t i = 0; i <= MAIN_APP_PAGE_END - MAIN_APP_PAGE_START; i++) {
        /* Start erasing 0x08008000 + 0x800 * 0 = 0x08008000 */
        flash_erase_page(MAIN_APP_START_ADDR + (MEMORY_PAGE_SIZE * i));
        /* End erasing 0x08008000 + 0x800 * 47 = 0x0801F800 */
    }
    flash_lock();
}

/**
 * @brief Reset Meta Data Region and Set sentinel to NOTOK
 * 
 */
void flashif_eraseAppMetaData(void) {
    flash_unlock();
    flash_erase_page(AMD_START_ADDR);
    flash_program_word(AMD_START_ADDR, FW_SENTINEL_NOTOK);
    flash_lock();
}

/**
 * @brief Write data into the metadata region
 * 
 * @param metadata Pointer to metadata
 */
void flashif_writeAppMetaData(fwMeta_ts* metadata) {
    flash_unlock();
    flash_program_word(AMD_START_ADDR, metadata->sentinel);
    flash_program_word(AMD_START_ADDR + 4, metadata->device_id);
    flash_program_word(AMD_START_ADDR + 8, metadata->version);
    flash_program_word(AMD_START_ADDR + 12, metadata->length);
    flash_program_word(AMD_START_ADDR + 16, metadata->crc32);
    flash_lock();
}

/**
 * @brief write data into flash 
 * 
 * @param address Start address of 32-bit write (address ends in 0 4 8 C)
 * @param data pointer to array of bytes
 * @param length length of 1 byte data to write
 */
bool flashif_write(const uint32_t address, const uint8_t* data, const uint32_t length) {
    #ifdef BOOTLOADER
    // Simple protection for bootloader area
    if (address < MAIN_APP_START_ADDR) {
        return false;
    }
    #endif // BOOTLOADER
    
    // Check 32-bit alignment
    if (address & 0b11) return false;

    uint32_t i = 0;
    uint32_t data32 = 0xFFFFFFFF;
    
    flash_unlock();
    /* Write 32 bit */
    while (i + 4 <= length) {
        data32 = data[i] | (data[i+1] << 8) | (data[i+2] << 16) | (data[i+3] << 24);
        flash_program_word(address + i, data32);
        i += 4;
    }
    
    /* Account for non 32 bit data */
    if (length - i) {
        switch (length - i) {
            case 1: {
                data32 = data[i] | (0xFFFFFF00);   
            } break;
            case 2: {
                data32 = data[i] | (data[i+1] << 8) | 0xFFFF0000;
            } break;
            case 3: {
                data32 = data[i] | (data[i+1] << 8) | (data[i+2] << 16) | 0xFF000000;
            } break;
            default:
                break;
        }       
        flash_program_word(address + i, data32);
    }

    flash_lock();

    return true;
}
