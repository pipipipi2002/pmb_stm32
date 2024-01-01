#ifndef INC_METADATA_H
#define INC_METADATA_H

#include "common_defines.h"
#include "board_defines.h"

#define FW_SENTINEL_OK            (0x6C0FFEE9)
#define FW_SENTINEL_NOTOK         (0xDEADBEEF)

typedef struct {
    uint32_t sentinel;
    uint32_t device_id;
    uint32_t version;
    uint32_t length;
    uint32_t crc32;
} fwMeta_ts;

#endif // INC_METADATA_H