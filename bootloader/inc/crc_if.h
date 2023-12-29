#ifndef INC_CRC_IF_H
#define INC_CRC_IF_H

#include "common_defines.h"

bool crcif_setup(void);
bool crcif_destruct(void);
uint32_t crcif_compute32(uint8_t* data, uint32_t length);

#endif // INC_CRC_IF_H