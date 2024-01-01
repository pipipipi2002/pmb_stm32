// Libopencm3 Header Files
#include <libopencm3/stm32/gpio.h>

// Shared Header Files
#include "common_defines.h"
#include "metadata.h"
#include "bootloader_defines.h"
#include "board_defines.h"
#include "system.h"
#include "log.h"
#include "manager.h"
#include "uart_if.h"
#include "gpio_if.h"
#include "can_if.h"
#include "crc_if.h"
#include "flash_if.h"

#ifndef BOOTLOADER
    #error "BOOTLOADER OPTION NOT SELECTED"
#endif

typedef enum {
    BL_FUR_STATE,
    BL_DEVID_STATE,
    BL_FW_LENGTH_STATE,
    BL_FW_CRC_STATE,
    BL_FW_VER_STATE,
    BL_ERASE_APP_STATE,
    BL_RECV_FW_STATE,
    BL_DONE_STATE,
} bl_state_te;

/*
 * Global variables
 */

/**
 * @brief App meta data which is only written during fw flashing.
 *      During Fw Flashing, appMetaData will be cleared together with app.
 *      This information will be used for validation before jumping to app.
 *      If calculated crc is not equal to stored crc, the meta data will be 
 *      cleared and sentinel set to NOTOK. 
 */
__attribute__((section (".fw_meta"))) fwMeta_ts appMetaData = {
    .sentinel = FW_SENTINEL_NOTOK,      // Validity (values are kept in boot)
    .device_id = DEVICE_ID,             // Device ID (values are kept in boot)
    .version = 0xFFFFFFFF,              // git commit version -> FROM SERVER
    .length = 0xFFFFFFFF,               // Firmware Length -> FROM SERVER
    .crc32 = 0xFFFFFFFF,                // CRC-32 of the firmware -> FROM SERVER
};
static uint32_t fwLength = 0;
static uint32_t fwBytesWritten = 0;
static uint32_t fwCRC = 0;
static uint32_t fwVersion = 0;
static bl_state_te state = BL_FUR_STATE;
static man_packet_ts packet_tx, packet_rx;
static canMsg_tu canHbMsg = {.heartbeatId = BB_HEARTBEAT_ID_PMB};
static timeout_ts masterTime, canHbTime;

/*
 * Internal (static) Function Declarations
 */
int main (void);
static void setup(void);
static void destruct(void);
static void jumpToApplication(void);
static bool validateApplication(void);

/*
 * Function Definitions
 */
int main (void) {
    system_rccInit();
    system_systickInit();
    setup();
    log_pInfo("Power Monitoring Board AUV4, Bootloader OK ");
    gpio_clear(PMB_NERROR_PORT, PMB_NERROR_PIN);

    /* Supply power */
    gpio_clear(PMB_RELAY_OFF_PORT, PMB_RELAY_OFF_PIN);          // Latch pwoer to PMB 
    gpio_clear(PMB_PMOS_ON_GPIO_PORT, PMB_PMOS_ON_GPIO_PIN);    // Disable power to vehicle
    log_pInfo("Latched power to circuit, disabled output power");

    /* Bootloader preparation */
    timeout_setup(&canHbTime, CAN_HB_TIMEOUT, true);
    timeout_setup(&masterTime, MASTER_TIMEOUT, false);

    log_pInfo("Entering SYNC State");

    while (state != BL_DONE_STATE) {
        if (timeout_hasElapsed(&masterTime) == true) {
            log_pInfo("Bootloader Timeout");
            state = BL_DONE_STATE;
            break;
        }

        switch (state) {
            case BL_FUR_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    /* Check for FUR data */
                    if (man_isBLPacketSingle(&packet_rx, BL_FUR_REQ_PACKET)) {
                        log_pInfo("Received FUR Packet");
                        /* Send FUR ACK */
                        man_createBLPacketSingle(&packet_tx, BL_FUR_ACK_RES_PACKET);
                        man_write(&packet_tx);
                        log_pInfo("Sent FUR ACK");
                        /* Transition to DEVID state */
                        state = BL_DEVID_STATE;
                        log_pInfo("Enter DEVID State");
                        /* Reset Time */
                        timeout_reset(&masterTime);
                    } else {
                        log_pError("Received unknown packet in FUR state");
                    }
                }
                /* Send Heartbeat */
                if (timeout_hasElapsed(&canHbTime) == true) {
                    canif_sendVehMsg(&canHbMsg, BB_CAN_HB_MSG_SIZE, BB_CAN_ID_HEARTBEAT);
                }
            } break;
            case BL_DEVID_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    /* Check for DEVID data */
                    if (man_isBLPacketData(&packet_rx, BL_DEVID_REQ_PACKET, BL_PMB_DEVID_MSG)) {
                        log_pInfo("Received DEVID Packet");
                        /* Send DEVID ACK */
                        man_createBLPacketData(&packet_tx, BL_DEVID_ACK_RES_PACKET, BL_PMB_DEVID_MSG);
                        man_write(&packet_tx);
                        log_pInfo("Sent DEVID ACK");
                        /* Transition to DEVID state */
                        state = BL_FW_LENGTH_STATE;
                        log_pInfo("Enter FWLEN State");
                        /* Reset Time */
                        timeout_reset(&masterTime);
                    } else {
                        log_pError("Received unknown packet in DEVID state");
                        state = BL_DONE_STATE;
                        man_sendNack();
                        break;
                    }
                }
            } break;
            case BL_FW_LENGTH_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    fwLength = packet_rx.data[1] | (packet_rx.data[2] << 8) | (packet_rx.data[3] << 16) | (packet_rx.data[4] << 24);
                    if (man_isBLPacketData(&packet_rx, BL_FWLEN_REQ_PACKET, 0) && (fwLength <= MAIN_APP_SIZE_MAX)) {
                        log_pInfo("Received FWLEN Packet: %u Bytes", fwLength);
                        /* Send FW Len ACK */
                        man_createBLPacketData(&packet_tx, BL_FWLEN_ACK_RES_PACKET, fwLength);
                        man_write(&packet_tx);
                        log_pInfo("Sent FWLEN ACK");
                        /* Transition to FW CRC state */
                        state = BL_FW_CRC_STATE;
                        log_pInfo("Enter FW CRC State");
                        /* Reset Time */
                        timeout_reset(&masterTime);
                    } else {
                        log_pError("Received unknown packet in FW CRC state");
                        state = BL_DONE_STATE;
                        man_sendNack();
                        break;
                    }
                }
            } break;

            case BL_FW_CRC_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    if (man_isBLPacketData(&packet_rx, BL_FWCRC_REQ_PACKET, 0)) {
                        fwCRC = packet_rx.data[1] | (packet_rx.data[2] << 8) | (packet_rx.data[3] << 16) | (packet_rx.data[4] << 24);
                        log_pInfo("Received FW CRC Packet: 0x%X", fwCRC);
                        /* Send FW CRC ACK */
                        man_createBLPacketData(&packet_tx, BL_FWCRC_ACK_RES_PACKET, fwCRC);
                        man_write(&packet_tx);
                        log_pInfo("Sent FW CRC ACK");
                        /* Transition to VER App state */
                        state = BL_FW_VER_STATE;
                        log_pInfo("Enter FW VER State");
                        /* Reset Time */
                        timeout_reset(&masterTime);
                    } else {
                        log_pError("Received unknown packet in FW VER state");
                        state = BL_DONE_STATE;
                        man_sendNack();
                        break;
                    }
                }
            } break;

            case BL_FW_VER_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    if (man_isBLPacketData(&packet_rx, BL_FWVER_REQ_PACKET, 0)) {
                        fwVersion = packet_rx.data[1] | (packet_rx.data[2] << 8) | (packet_rx.data[3] << 16) | (packet_rx.data[4] << 24);
                        log_pInfo("Received FW Version Packet: 0x%X", fwVersion);
                        /* Send FW VER ACK */
                        man_createBLPacketData(&packet_tx, BL_FWVER_ACK_RES_PACKET, fwVersion);
                        man_write(&packet_tx);
                        log_pInfo("Sent FW VER ACK");
                        /* Transition to Erase App state */
                        state = BL_ERASE_APP_STATE;
                        log_pInfo("Enter ERASE APP State");
                        /* Reset Time */
                        timeout_reset(&masterTime);
                    } else {
                        log_pError("Received unknown packet in FW VER state");
                        state = BL_DONE_STATE;
                        man_sendNack();
                        break;
                    }
                }
            } break;

            case BL_ERASE_APP_STATE: {
                /* Erase main application */
                flashif_eraseAppMetaData();
                flashif_eraseMainApplication();

                /* Write data to META */
                fwMeta_ts meta = {
                    .sentinel = FW_SENTINEL_OK,
                    .device_id = BL_PMB_DEVID_MSG,
                    .version = fwVersion,
                    .length = fwLength,
                    .crc32 = fwCRC
                };
                flashif_writeAppMetaData(&meta);

                /* Transition to Receive FW state */
                state = BL_RECV_FW_STATE;
                /* Clear FW Data Buffer */
                canif_clearQueue(CAN_ID_BOOTLOADER_DATA);

                /* Send Data Ready */
                man_createBLPacketSingle(&packet_tx, BL_RECVRDY_RES_PACKET);
                man_write(&packet_tx);    
                log_pInfo("Sent Receive Ready Packet");
                /* Reset Time */
                timeout_reset(&masterTime);
            } break;

            case BL_RECV_FW_STATE: {
                /* Wait for Data Sent Packet */
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    /* Check Data Sent Packet */
                    if (!man_isBLPacketData(&packet_rx, BL_DATASENT_REQ_PACKET, 0)) {
                        log_pError("Received unknown packet in RECV FW state");
                        state = BL_DONE_STATE;
                        man_sendNack();
                        break;
                    }
                    /* Segment size + bytes written will never go above firmware size as we have confirmed in the previous stae */
                    uint32_t rxSegmentSize = packet_rx.data[1] | (packet_rx.data[2] << 8) | (packet_rx.data[3] << 16) | (packet_rx.data[4] << 24);
                    uint8_t* segmentBuffer = canif_getQueuePointer(CAN_ID_BOOTLOADER_DATA);
                    log_pInfo("Received Data Sent Packet: %u Bytes", rxSegmentSize);

                    /* Write to flash */
                    if (!flashif_write(MAIN_APP_START_ADDR + fwBytesWritten, segmentBuffer, rxSegmentSize)) {
                        state = BL_DONE_STATE;
                        man_sendNack();
                        break;
                    }
                    fwBytesWritten += rxSegmentSize;
                    log_pInfo("Written %u/%u Bytes", fwBytesWritten, fwLength);

                    /* Check for completion */
                    if (fwBytesWritten >= fwLength) {
                        man_createBLPacketSingle(&packet_tx, BL_SUCCESS_RES_PACKET);
                        man_write(&packet_tx);
                        log_pInfo("Received all data");
                        state = BL_DONE_STATE;
                    } else {
                        /* Clear FW Data Buffer */
                        canif_clearQueue(CAN_ID_BOOTLOADER_DATA);
                        /* Send Data Ready */
                        man_createBLPacketSingle(&packet_tx, BL_RECVRDY_RES_PACKET);
                        man_write(&packet_tx);                        
                    }
                    /* Reset Time */
                    timeout_reset(&masterTime);
                }
            } break;

            default: {
                state = BL_FUR_STATE;
            } break;
        }
        man_update();
    }

    /* BL_DONE_STATE */

    /* Jump to App */
    jumpToApplication();

    while(1); // Spin forever
    return 0; // will not reach
}

static void setup(void) {
    while(!uart1if_setup()) system_delayMs(1000);
    while(!gpioif_setup()) system_delayMs(1000);
    while(!canif_setup()) system_delayMs(1000);
    crcif_setup();
    man_setup();
    log_pSuccess("Setup Completed");
}
static void destruct(void) {
    crcif_destruct();
    uart1if_destruct();
    system_systickDeinit();
    gpio_set(PMB_NERROR_PORT, PMB_NERROR_PIN);
}

static bool validateApplication(void) {
    if (appMetaData.sentinel == FW_SENTINEL_NOTOK) {
        log_pError("Sentinel Not Ok");
        return false;
    }

    if (appMetaData.device_id != BL_PMB_DEVID_MSG) {
        log_pError("Device ID invalid: 0x%X", appMetaData.device_id);
        return false;
    }

    uint8_t* fwStartAddr = (uint8_t*) MAIN_APP_START_ADDR;
    uint32_t computedCrc = crcif_compute32(fwStartAddr, appMetaData.length);
    if (computedCrc != appMetaData.crc32) {
        log_pError("App crc wrong. Expected: 0x%X, Computed: 0x%X", appMetaData.crc32, computedCrc);
        return false;
    }

    log_pSuccess("Main App CRC OK");
    log_pInfo("Using version: 0x%X", appMetaData.version);
    return true;
}

static void jumpToApplication(void) {
    /* Validate App */
    if (!validateApplication()) return;

    /* Teardown */
    destruct();

    /* Jump to Main App */
    typedef void (*void_fn)(void); // Function Pointer

    /* vector table is after SP; Reset handler is 1st in the Vector Table */
    uint32_t* reset_vector_addr = (uint32_t*) (MAIN_APP_START_ADDR + 4U);
    /* The reset handler contains the address to the reset function */ 
    uint32_t* reset_handler_addr = (uint32_t*) (*reset_vector_addr); 
    /* Map reset vector to a function */
    void_fn app_reset_handler = (void_fn) reset_handler_addr;
    /* Set MSP to the APP stack pointer */
    uint32_t app_top_stack = (*(uint32_t*)MAIN_APP_START_ADDR); 
    __asm volatile ("MSR msp, %0" : : "r" (app_top_stack) : );
    /* Jump to Main Application Reset Handler */
    app_reset_handler();
}
