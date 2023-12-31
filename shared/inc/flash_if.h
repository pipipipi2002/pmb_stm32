#ifndef INC_FLASH_IF_H
#define INC_FLASH_IF_H

#include "common_defines.h"
#include "metadata.h"

void flashif_eraseMainApplication(void);
void flashif_writeAppMetaData(fwMeta_ts* metadata);
void flashif_eraseAppMetaData(void);
bool flashif_write(const uint32_t address, const uint8_t* data, const uint32_t length);

#endif // INC_FLASH_IF_H