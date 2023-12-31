#ifndef INC_BOOTLOADER_DEFINES_H
#define INC_BOOTLOADER_DEFINES_H

#include "common_defines.h"
#include "board_def.h"

#if !defined (DEVICE_ID)
    #error "Firmware ID not defined"
#endif // FW_DEVICE_ID

#define CAN_ID_BOOTLOADER               (40U)
#define CAN_ID_BOOTLOADER_DATA          (41U)

/* 4-Byte Msg definition for the flash state machine */
// REQ: Server -> PMB
// RES: PMB -> Server

#define BL_FUR_REQ_PACKET               (0xCA)
#define BL_FUR_ACK_RES_PACKET           (0xBD)

#define BL_DEVID_REQ_PACKET             (0x36)
#define BL_DEVID_ACK_RES_PACKET         (0x42)
#define BL_PMB_DEVID_MSG                (DEVICE_ID)

#define BL_FWLEN_REQ_PACKET             (0x39)
#define BL_FWLEN_ACK_RES_PACKET         (0x73)

#define BL_FWCRC_REQ_PACKET             (0x53)
#define BL_FWCRC_ACK_RES_PACKET         (0x64)

#define BL_FWVER_REQ_PACKET             (0x59)
#define BL_FWVER_ACK_RES_PACKET         (0x86)

#define BL_RECVRDY_RES_PACKET           (0xC0)
#define BL_DATASENT_REQ_PACKET          (0x88)

#define BL_SUCCESS_RES_PACKET           (0x69)

#define BL_FW_DATA_SEG_SIZE             (1024)

#endif // INC_BOOTLOADER_DEFINES_H