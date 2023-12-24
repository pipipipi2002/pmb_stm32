#include <libopencm3/stm32/crc.h>
#include "crc_if.h"

/**
 * @brief Compute 32bit crc based on CRC-32 Ethernet
 * 
 * @param data pointer to data
 * @param length Length of 8-bit data to process
 * @return uint32_t 
 */
uint32_t crcif_compute32(uint8_t* data, uint32_t length) {
    crc_reset(); // Resets to default CRC-32 Ethernet
    uint32_t rem = length % 4;
    uint32_t full_32_bit = length / 4;

    uint32_t i = 0;
    while (i < full_32_bit) {
        uint32_t data_32 = (data[4*i] << 24) | (data[4*i + 1] << 16) | (data[4*i + 2] << 8) | data[4*i + 3];
        CRC_DR = data_32;
        i++;
    }
    
    uint32_t temp_data = 0;
    switch (rem) {
        case 3: {
            temp_data += data[4*i + 2];
            __attribute__ ((fallthrough)); // Fall through expected
        }
        case 2: {
            temp_data += data[4*i + 1];
            __attribute__ ((fallthrough)); // Fall through expected
        }
        case 1: {
            temp_data += data[4*i];
            CRC_DR = temp_data;
            break;
        }
        default :
            break;
    }
    return CRC_DR;
}