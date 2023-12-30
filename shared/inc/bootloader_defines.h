#ifndef INC_BOOTLOADER_DEFINES_H
#define INC_BOOTLOADER_DEFINES_H

#include "common_defines.h"

#define CAN_ID_BOOTLOADER               (40U)
#define CAN_ID_BOOTLOADER_DATA          (41U)

/* 4-Byte Msg definition for the flash state machine */
// REQ: Server -> PMB
// RES: PMB -> Server

#define BL_SYNC_REQ_PACKET              (0xAA)
#define BL_SYNCED_RES_PACKET            (0x55)

#define BL_FUR_REQ_PACKET               (0xCA)
#define BL_FUR_ACK_RES_PACKET           (0xBD)

#define BL_DEVID_REQ_PACKET             (0x36)
#define BL_DEVID_ACK_RES_PACKET         (0x42)
#define BL_PMB_DEVID_MSG                (0xBEEF6969)

#define BL_FWLEN_REQ_PACKET             (0x39)
#define BL_FWLEN_ACK_RES_PACKET         (0x73)

#define BL_DATARDY_RES_PACKET           (0xC0)
#define BL_DATASENT_REQ_PACKET          (0x88)

#define BL_SUCCESS_RES_PACKET           (0x69)

#define BL_FW_DATA_SEG_SIZE             (1024)

#endif // INC_BOOTLOADER_DEFINES_H