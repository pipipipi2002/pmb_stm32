// Libopencm3 Header Files
#include <libopencm3/stm32/gpio.h>

#include <assert.h>

// Shared Header Files
#include "common_defines.h"
#include "board_def.h"
#include "system.h"
#include "log.h"
#include "uart_if.h"
#include "gpio_if.h"
#include "can_if.h"
#include "bootloader_defines.h"
#include "manager.h"
#include "crc_if.h"
#include "flash_if.h"

#ifndef BOOTLOADER
    #error "BOOTLOADER OPTION NOT SELECTED"
#endif

#define MASTER_TIMEOUT          (5000U)
#define CAN_HB_TIMEOUT          (1000U)

typedef enum {
    BL_SYNC_STATE,
    BL_FUR_STATE,
    BL_DEVID_STATE,
    BL_FW_LENGTH_STATE,
    BL_ERASE_APP_STATE,
    BL_RECV_FW_STATE,
    BL_DONE_STATE
} bl_state_te;

/*
 * Global variables
 */
static bl_state_te state = BL_SYNC_STATE;
static uint32_t fwLength = 0;
static uint32_t fwBytesWritten = 0;
static man_packet_ts packet_tx, packet_rx, nack;
static canMsg_tu canHbMsg = {.heartbeatId = BB_HEARTBEAT_ID_PMB};

static timeout_ts masterTime, canHbTime;
/*
 * Internal Function Declarations
 */
int main (void);
static void setup(void);
static void destruct(void);
static void jumpToApplication(void);

/*
 * Function Definitions
 */
int main (void) {
    system_rccInit();
    system_systickInit();
    setup();
    log_pInfo("Power Monitoring Board AUV4, Bootloader OK ");

    /* Supply power */
    gpio_clear(PMB_RELAY_OFF_PORT, PMB_RELAY_OFF_PIN);          // Latch pwoer to PMB 
    gpio_clear(PMB_PMOS_ON_GPIO_PORT, PMB_PMOS_ON_GPIO_PIN);    // Disable power to vehicle
    log_pInfo("Latched power to circuit, disabled output power");

    /* Bootloader preparation */
    timeout_setup(&canHbTime, CAN_HB_TIMEOUT, true);
    timeout_setup(&masterTime, MASTER_TIMEOUT, false);
    man_createBLPacketSingle(&nack, BL_NACK_PACKET);

    while (state != BL_DONE_STATE) {
        if (timeout_hasElapsed(&masterTime) == true) {
            log_pInfo("Bootloader Timeout");
            state = BL_DONE_STATE;
            break;
        }

        switch (state) {
            case BL_SYNC_STATE: {
                /* Check for SYNC packet */
                if (man_packetAvailable()) {
                    man_read(&packet_rx);
                    
                    /* Check SYNC data */
                    if (man_isBLPacketSingle(&packet_rx, BL_SYNC_REQ_PACKET)) {
                        /* Send SYNCED msg */
                        man_createBLPacketSingle(&packet_tx, BL_SYNCED_RES_PACKET);
                        man_write(&packet_tx);
                        /* Transition to FUR state */
                        state = BL_FUR_STATE;
                        /* Reset Time */
                        timeout_reset(&masterTime);
                        break;
                    } // Wait for correct sync until timeout
                }
                /* Send Heartbeat */
                if (timeout_hasElapsed(&canHbTime) == true) {
                    canif_sendVehMsg(&canHbMsg, BB_CAN_HB_MSG_SIZE, BB_CAN_ID_HEARTBEAT);
                }
            } break;
            case BL_FUR_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    /* Check for FUR data */
                    if (man_isBLPacketSingle(&packet_rx, BL_FUR_REQ_PACKET)) {
                        /* Send FUR ACK */
                        man_createBLPacketSingle(&packet_tx, BL_FUR_ACK_RES_PACKET);
                        man_write(&packet_tx);
                        /* Transition to DEVID state */
                        state = BL_DEVID_STATE;
                        /* Reset Time */
                        timeout_reset(&masterTime);
                    } else {
                        state = BL_DONE_STATE;
                        man_write(&nack);
                        break;
                    }
                }
            } break;
            case BL_DEVID_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    /* Check for DEVID data */
                    if (man_isBLPacketData(&packet_rx, BL_DEVID_REQ_PACKET, BL_PMB_DEVID_MSG)) {
                        /* Send DEVID ACK */
                        man_createBLPacketData(&packet_tx, BL_DEVID_ACK_RES_PACKET, BL_PMB_DEVID_MSG);
                        man_write(&packet_tx);
                        /* Transition to DEVID state */
                        state = BL_FW_LENGTH_STATE;
                        /* Reset Time */
                        timeout_reset(&masterTime);
                    } else {
                        state = BL_DONE_STATE;
                        man_write(&nack);
                        break;
                    }
                }
            } break;
            case BL_FW_LENGTH_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    fwLength = packet_rx.data[1] | (packet_rx.data[2] << 8) | (packet_rx.data[3] << 16) | (packet_rx.data[4] << 24);
                    if (man_isBLPacketData(&packet_rx, BL_FWLEN_REQ_PACKET, 0) && (fwLength <= MAIN_APP_SIZE_MAX)) {
                        /* Send FW Len ACK */
                        man_createBLPacketData(&packet_tx, BL_FWLEN_ACK_RES_PACKET, fwLength);
                        man_write(&packet_tx);
                        /* Transition to Erase App state */
                        state = BL_ERASE_APP_STATE;
                        /* Reset Time */
                        timeout_reset(&masterTime);
                    } else {
                        state = BL_DONE_STATE;
                        man_write(&nack);
                        break;
                    }
                }
            } break;

            case BL_ERASE_APP_STATE: {
                /* Erase main application */
                flashif_eraseMainApplication();
                /* Transition to Receive FW state */
                state = BL_RECV_FW_STATE;
                /* Send Data Ready */
                man_createBLPacketSingle(&packet_tx, BL_DATARDY_RES_PACKET);
                man_write(&packet_tx);    
                /* Reset Time */
                timeout_reset(&masterTime);
            } break;

            case BL_RECV_FW_STATE: {
                if (man_packetAvailable()) {
                    man_read(&packet_rx);

                    /* Always write 8 bytes as chip only supports half-word/word write */
                    if (!flashif_write(MAIN_APP_START_ADDR + fwBytesWritten, packet_rx.data, 8)) {
                        state = BL_DONE_STATE;
                        man_write(&nack);
                        break;
                    }
                    fwBytesWritten += 8;

                    /* Check for completion */
                    if (fwBytesWritten >= fwLength) {
                        man_createBLPacketSingle(&packet_tx, BL_SUCCESS_RES_PACKET);
                        man_write(&packet_tx);
                        state = BL_DONE_STATE;
                    } else {
                        /* Send Data Ready */
                        man_createBLPacketSingle(&packet_tx, BL_DATARDY_RES_PACKET);
                        man_write(&packet_tx);                        
                    }
                    /* Reset Time */
                    timeout_reset(&masterTime);
                }
            } break;

            default: {
                state = BL_SYNC_STATE;
            } break;
        }
        man_update();
    }

    /* BL_DONE_STATE */

    /* Firmware validation */

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
}

static void jumpToApplication(void) {
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
