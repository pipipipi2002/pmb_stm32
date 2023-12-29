#ifndef INC_BOOTLOADER_DEFINES_H
#define INC_BOOTLOADER_DEFINES_H

#include "common_defines.h"

#define CAN_ID_BOOTLOADER_SERVER        (40U)
#define CAN_ID_BOOTLOADER_PMB           (41U)

/* 4-Byte Msg definition for the flash state machine */
// REQ: Server -> PMB
// RES: PMB -> Server

#define BL_SYNC_REQ_PACKET          (0xAA)
#define BL_SYNCED_RES_PACKET        (0x55)

#define BL_FUR_REQ_PACKET           (0xCA)
#define BL_FUR_ACK_RES_PACKET       (0xBD)

#define BL_DEVID_REQ_PACKET         (0x36)
#define BL_DEVID_ACK_RES_PACKET     (0x42)
#define BL_PMB_DEVID_MSG            (0xBEEF6969)

#define BL_FWLEN_REQ_PACKET         (0x39)
#define BL_FWLEN_ACK_RES_PACKET     (0x73)

#define BL_DATARDY_RES_PACKET       (0xC0)

#define BL_SUCCESS_RES_PACKET       (0x69)

#define BL_NACK_PACKET              (0xDE)


#define BOOTLOADER_SERVER_JUMP_CMD      0xAA
#define BOOTLOADER_SERVER_ECHO_CMD      0xBB

#define WAIT_FOR_BOOT_SERVER_MS         (10000UL)

#endif // INC_BOOTLOADER_DEFINES_H