CAN_HEARTBEAT_ID                = 4
CAN_PMB1_HEARTBEAT_ID           = 7
CAN_PMB2_HEARTBEAT_ID           = 8

CAN_BOOTLOADER_SERVER_ID        = 40
CAN_BOOTLOADER_PMB_ID           = 41

PACKET_LENTYPE_SIZE             = 1
PACKET_DATA_SIZE                = 19
PACKET_CRC_SIZE                 = 4
PACKET_TOTAL_SIZE               = PACKET_LENTYPE_SIZE + PACKET_DATA_SIZE + PACKET_CRC_SIZE 

PACKET_TYPE_NORMAL              = 1
PACKET_TYPE_UTILITY             = 2
PACKET_TYPE_UTILITY_SIZE        = 1

PACKET_UTILITY_ACK_DATA         = 0x35
PACKET_UTILITY_RETX_DATA        = 0x67
PACKET_UTILITY_NACK_DATA        = 0xDE



BL_SYNC_REQ_PACKET              = 0xAA
BL_SYNCED_RES_PACKET            = 0x55
BL_FUR_REQ_PACKET               = 0xCA
BL_FUR_ACK_RES_PACKET           = 0xBD
BL_DEVID_REQ_PACKET             = 0x36
BL_DEVID_ACK_RES_PACKET         = 0x42
BL_PMB_DEVID_MSG                = 0xBEEF6969
BL_FWLEN_REQ_PACKET             = 0x39
BL_FWLEN_ACK_RES_PACKET         = 0x73
BL_DATARDY_RES_PACKET           = 0xC0
BL_UPDATE_FW_PACKET_DATA_SIZE   = 16
BL_SUCCESS_RES_PACKET           = 0x69
BL_NACK_PACKET                  = 0xDE

BOOTLOADER_SIZE                 = 0x8000

FW_FILE                         = "../app/firmware.bin"
SLEEP_TIME                      = 0.1 # interval between can frames
LOG_FILE                        = "fw-updater.log"