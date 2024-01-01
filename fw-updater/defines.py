CAN_BOOTLOADER_ID               = 40
CAN_BOOTLOADER_DATA_ID          = 41
CAN_HEARTBEAT_ID                = CAN_BOOTLOADER_DATA_ID
CAN_PMB1_HEARTBEAT_ID           = 7
CAN_PMB2_HEARTBEAT_ID           = 8


PACKET_LENTYPE_SIZE             = 1
PACKET_DATA_SIZE                = 6
PACKET_CRC_SIZE                 = 1
PACKET_TOTAL_SIZE               = PACKET_LENTYPE_SIZE + PACKET_DATA_SIZE + PACKET_CRC_SIZE 

PACKET_TYPE_NORMAL              = 1
PACKET_TYPE_UTILITY             = 2
PACKET_TYPE_UTILITY_SIZE        = 1

PACKET_UTILITY_ACK_DATA         = 0x35
PACKET_UTILITY_RETX_DATA        = 0x67
PACKET_UTILITY_NACK_DATA        = 0xDE

BL_FUR_REQ_PACKET               = 0xCA
BL_FUR_ACK_RES_PACKET           = 0xBD
BL_DEVID_REQ_PACKET             = 0x36
BL_DEVID_ACK_RES_PACKET         = 0x42
BL_FWLEN_REQ_PACKET             = 0x39
BL_FWLEN_ACK_RES_PACKET         = 0x73
BL_FWCRC_REQ_PACKET             = 0x53
BL_FWCRC_ACK_RES_PACKET         = 0x64
BL_FWVER_REQ_PACKET             = 0x59
BL_FWVER_ACK_RES_PACKET         = 0x86
BL_RECVRDY_RES_PACKET           = 0xC0
BL_DATASENT_REQ_PACKET          = 0x88
BL_SUCCESS_RES_PACKET           = 0x69

BOOTLOADER_SIZE                 = 30 * 1024
METADATA_SIZE                   = 2 * 1024
FIRMWARE_SIZE                   = 96 * 1024
BL_FW_DATA_SEG_SIZE             = 1024
FIRMWARE_INFO_OFFSET            = 0xc0
FIRMWARE_INFO_SIZE              = 8
FIRMWARE_INFO_COMMIT_OFFSET     = FIRMWARE_INFO_OFFSET + 4

FW_FILE                         = "../app/firmware.bin"
SLEEP_TIME                      = 0.001       # interval between can frames
LOG_FILE                        = "fw-updater.log"