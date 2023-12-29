#include "flash_if.h"
#include <libopencm3/stm32/flash.h>

// There are 63 Page of 2 KiBytes each. 1 sector makes up 2 page
// Bootloader takes up 32kiB -> 16 page (page 0 - 15)

// Main flash memory can be programmed 16 bits at a time (half word)
// Only write if the addressed has been erased (unless 0x0000 is being programmed)
// Addressed must not be write-protected by the FLASH_WRPR register

#define MAIN_APP_PAGE_START     (16)
#define MAIN_APP_PAGE_END       (63)
#define MEMORY_PAGE_SIZE        (0x800)

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
 * @brief write data into flash (minimum 4 byte)
 * 
 * @param address Start address of 32-bit write
 * @param data pointer to array of 1 byte data
 * @param length length of 1 byte data to write
 */
bool flashif_write(const uint32_t address, const uint8_t* data, const uint32_t length) {
    // Simple protection against bootloader area
    #ifdef BOOTLOADER
    if (address < MAIN_APP_START_ADDR) {
        return false;
    }
    #endif // BOOTLOADER
    
    flash_unlock();
    for (uint32_t i = 0; i < length/4; i++) {
        uint32_t data32 = data[i*4] | (data[i*4+1] << 8) | (data[i*4+2] << 16) | (data[i*4+3] << 24);
        flash_program_word(address + (i * 4), data32);
    }
    flash_lock();

    return true;
}
