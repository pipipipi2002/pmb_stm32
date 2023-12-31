#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/rcc.h>
#include "crc_if.h"

static uint8_t crcif_reverseBit(uint8_t byte);

bool crcif_setup(void) {
    rcc_periph_clock_enable(RCC_CRC);
    return true;
}

bool crcif_destruct(void) {
    rcc_periph_clock_disable(RCC_CRC);
    return true;
}

uint8_t crcif_compute8(uint8_t* data, uint32_t length) {
    crc_reset();
    crc_set_polysize(CRC_CR_POLYSIZE_8);
    crc_set_polynomial(0x07);
    crc_set_initial(0);
    crc_set_reverse_input(CRC_CR_REV_IN_NONE);
    crc_reverse_output_disable();

    for (uint32_t i = 0; i < length; i++) {
        CRC_DR8 = data[i];
    }

    return CRC_DR8;
}

/**
 * @brief Compute 32bit crc based on CRC-32 Ethernet
 * 
 * @param data pointer to data
 * @param length Length of 8-bit data to process
 * @return uint32_t 
 */
uint32_t crcif_compute32(uint8_t* data, uint32_t length) {
    /* Sequence of CRC-32 Ethernet */
    // For a 32-bit input:
    // 1. per byte bit-reverse the input -> handled by hardware
    // 2. Implement CRC with the default polynomial -> handled by hardware
    // 3. Repeat 1 and 2 until no more data
    // 4. Now we have the output of CRC_DR.
    // 4a. per byte bit-reverse the input -> manual
    // 4b. XOR the result of 4a with 0xFFFFFFFF
    // Result: 32bit CRC with Little Byte Endian format

    
    /* Resets to default CRC-32 polynomial (ethernet) */
    crc_reset(); 
    crc_set_initial(0xFFFFFFFF);
    crc_set_polysize(CRC_CR_POLYSIZE_32);
    crc_set_polynomial(CRC_POL_DEFAULT);
    /* For each input, perform per Byte bit-reversal */
    crc_set_reverse_input(CRC_CR_REV_IN_BYTE);
    /* No reversal for output, as hardware only can reverse the entire word */
    crc_reverse_output_disable(); 

    uint32_t rem = length % 4;
    uint32_t full_32_bit = length / 4;
    uint32_t i = 0;
    /* Step 3 */
    // Calculate in multiple of 32
    while (i < full_32_bit) {
        uint32_t data_32 = (data[4*i] << 24) | (data[4*i + 1] << 16) | (data[4*i + 2] << 8) | data[4*i + 3];
        CRC_DR = data_32;
        i++;
    }

    // Calculate for less than 4bytes of data
    for (uint8_t j = 0; j < rem; j++) {
        uint8_t temp_data = data[4*i + j];
        CRC_DR8 = temp_data;
    }

    /* Split into bytes (little endian) */
    uint8_t res[4] = {(CRC_DR >> 24) & 0xFF, (CRC_DR >> 16) & 0xFF, (CRC_DR >> 8) & 0xFF, (CRC_DR) & 0xFF};
    /* Step 4a */
    for (uint8_t j = 0; j < 4; j++) {
        res[j] = crcif_reverseBit(res[j]);
    }

    /* Combine into word (big endian) */
    uint32_t wordRes = (res[0]) | (res[1] << 8) | (res[2] << 16) | (res[3] << 24);
    /* Step 4b */
    return wordRes ^ 0xFFFFFFFF;
}

static uint8_t crcif_reverseBit(uint8_t byte) {
    byte = (byte & 0x55) << 1 | (byte & 0xAA) >> 1;
    byte = (byte & 0x33) << 2 | (byte & 0xCC) >> 2;
    byte = (byte & 0x0F) << 4 | (byte & 0xF0) >> 4;
    return byte;
}
