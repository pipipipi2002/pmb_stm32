#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "board_def.h"
#include "can_if.h"
#include "cqueue.h"
#include "log.h"
#include "system.h"

static void canif_receive(uint8_t fifo);

#if defined (BOOTLOADER)
#include "bootloader_defines.h"

#define BOOT_SERVER_MSG_QUEUE_BUFFER_SIZE       (64)

static uint8_t rxBootServerBuffer[BOOT_SERVER_MSG_QUEUE_BUFFER_SIZE];
static cqueue_ts cq_rxCanBootServer = {0};

#elif defined (MAINAPP)
#define VEH_MSG_QUEUE_BUFFER_SIZE               (32)

static uint8_t rxVehBuffer[VEH_MSG_QUEUE_BUFFER_SIZE];
static cqueue_ts cq_rxCanVehMsg = {0};

#endif // Variant Switch

void cec_can_isr (void) {
    // Message pending on FIFO 0?
    if (CAN_RF0R(CAN1) & CAN_RF0R_FMP0_MASK) {
        canif_receive(0);
    }

    // Message pending on FIFO 1?
    if (CAN_RF1R(CAN1) & CAN_RF1R_FMP1_MASK) {
        canif_receive(1);
    }
}
/**
 * @brief Initialise the CAN1 port on PMB. This includes GPIO, RCC, CAN configs, filters
 * 
 * @return true if success
 * @return false if fail
 */
bool canif_setup(void) {
    log_pInfo("CAN Init");

    /* Initialise Queue for RX */
    #if defined (BOOTLOADER)
    cqueue_init(&cq_rxCanBootServer, rxBootServerBuffer, BOOT_SERVER_MSG_QUEUE_BUFFER_SIZE);
    #elif defined (MAINAPP)
    cqueue_init(&cq_rxCanVehMsg, rxVehBuffer, VEH_MSG_QUEUE_BUFFER_SIZE);
    #endif // Variant Switch

    /* Initialise CAN GPIO ports */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, PMB_CAN_RX_PIN | PMB_CAN_TX_PIN);
    gpio_set_af(GPIOA, GPIO_AF4, PMB_CAN_TX_PIN | PMB_CAN_RX_PIN);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PMB_CAN_RX_PIN | PMB_CAN_TX_PIN);
    
    /* Enable clock to CAN peripheral */
    rcc_periph_clock_enable(RCC_CAN);

    /* Reset CAN registers */
    can_reset(BX_CAN1_BASE);

    /* CAN1 Init */
    uint32_t ret = can_init(
        CAN1,               // CAN port
        false,              // TTCM
        true,               // ABOM
        false,              // AWUM
        false,              // NART
        false,              // RFLM
        false,              // TXFP
        CAN_BTR_SJW_1TQ,    // SJW
        CAN_BTR_TS1_13TQ,   // TS1
        CAN_BTR_TS2_2TQ,    // TS2
        3,                  // BRP (of AHB freq, 48 MHz)
        false,              // Loopback
        false);             // Silent

    if (ret == 1) {
        log_pError("CAN init failed");
        return false;
    }
    
    /* Enable Interrupt on CAN RX */
    can_enable_irq(CAN1, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
    nvic_enable_irq(NVIC_CEC_CAN_IRQ);


#if defined (MAINAPP)
	/* ID 0 ~ 15 goes to FIFO0
	 * Filter configuration:
	 * ID: 		0b00001111
	 * Mask:	0b11110000
	 * Effect:  Accept all IDs below 16
	 */
    const uint16_t id1 = (0b00001111 << 5);
    const uint16_t mask1 = (0b11110000 << 5);

    /* ID 16 ~  goes to FIFO1
	 * Filter configuration:
	 * ID: 		0x00
	 * Mask:	0x00
	 * Effect:  Accept all. But since 0~15 will go into filter bank with higher priority,
	 * only 16~ will go here
	 */
    const uint16_t id2 = 0;
    const uint16_t mask2 = (0b11111111 << 5);

#elif defined (BOOTLOADER) 
    /* ID 40-43 goes to FIFO0
	 * Filter configuration:
	 * ID: 		0b00101000
	 * Mask:	0b11111100
	 * Effect:  Accept only ID 40, 41, 42, 43
	 */
    const uint16_t id1 = (0b00101000 << 5);
    const uint16_t mask1 = (0b11111100 << 5);
    
    /* ID 0 ~ 15 goes to FIFO1
	 * Filter configuration:
	 * ID: 		0b00001111
	 * Mask:	0b11110000
	 * Effect:  Accept all IDs below 16
	 */
    const uint16_t id2 = (0b00001111 << 5);
    const uint16_t mask2 = (0b11110000 << 5);
#endif // Variant Switch

    can_filter_id_mask_16bit_init(1, id1, mask1, id1, mask1, 0, true); // FIFO0
    can_filter_id_mask_16bit_init(2, id2, mask2, id2, mask2, 1, true); // FIFO1

    log_pSuccess("CAN Init Sucessful");
    return true;
};

/**
 * @brief Send can message based on the defined can format. 
 * 
 * @param msg pointer to the msg to be sent
 * @param size bytes of data to send. Maximum 8.
 * @param msgId can msg id
 * @return int8_t number of bytes sent, -1 for failure
 */
int8_t canif_sendVehMsg (canMsg_tu* msg, uint8_t size, uint32_t msgId) {
    int8_t res = can_transmit(CAN1, msgId, false, false, size, msg->au8msg); // Non blocking tx
    return res;
}

/**
 * @brief Send can message with no limitation on bytes length (<255)
 * 
 * @param data pointer to the data buffer
 * @param size size of the data byte to send
 * @param id can id destination
 * @return int8_t number of bytes sent
 */

uint8_t canif_sendData (uint8_t* data, uint8_t size, uint32_t id) {
    uint8_t bytesSent = 0;
    uint8_t res = 0;

    while (size - bytesSent >= 8) {
        /* Send full 8 bytes */
        res += can_transmit(CAN1, id, false, false, 8, &(data[bytesSent]));
        bytesSent += 8;
        system_delayMs(1); // To prevent data dropped 
    }
    if (bytesSent != size) {
        /* Send remaining bytes */
        res += can_transmit(CAN1, id, false, false, (size - bytesSent), &(data[bytesSent]));
    }

    return res;
}

/**
 * @brief Receive routine for CAN
 * 
 * @param fifo FIFO ID, 0 or 1
 */
static void canif_receive(uint8_t fifo) {
    canFrame_ts rxFrame;
    bool ext, rtr;
    uint8_t fmi;
    uint16_t ts;
    can_receive(CAN1, fifo, true,
                &rxFrame.id,
                &ext,
                &rtr,
                &fmi,
                &rxFrame.len,
                rxFrame.data,
                &ts);

    #if defined (BOOTLOADER)
    switch (rxFrame.id) {
        case CAN_ID_BOOTLOADER_SERVER:
            uint8_t recv = cqueue_pushn(&cq_rxCanBootServer, rxFrame.data, rxFrame.len);
            if (recv != rxFrame.len) {
                log_pError("CAN Queue Full, dropped %d msgs.", rxFrame.len - recv);
            }
            break;
        default:
            log_pError("Invalid CAN ID received: %d", rxFrame.id);
            break;
    }
    #elif defined (MAINAPP)
    uint8_t recv = cqueue_pushn(&cq_rxCanVehMsg, rxFrame.data, rxFrame.len);
    if (recv != rxFrame.len) {
        log_pError("CAN Queue Full, dropped %d msgs.", rxFrame.len - recv);
    }
    #endif // Variant Switch
}

/**
 * @brief Getter function for Data Ready
 * 
 * @return true if Data is ready, false otherwise
 */
bool canif_getRxDataReady(void) {
    #if defined (BOOTLOADER)
    return !cqueue_isEmpty(&cq_rxCanBootServer);
    #elif defined (MAINAPP)
    return !cqueue_isEmpty(&cq_rxCanVehMsg);
    #endif // Variant Switch
}

/**
 * @brief Retrieve the latest data in the buffer
 * 
 * @param[out] frame pointer to can frame
 * @return true if data copied successfully
 */
bool canif_getRxData(uint8_t* data) {
    #if defined (BOOTLOADER)
    if (cqueue_pop(&cq_rxCanBootServer, data)) {
        return true;
    }
    #elif defined (MAINAPP)
    if (cqueue_pop(&cq_rxCanVehMsg, data)) {
        return true;
    }
    #endif // Variant Switch
    
    log_pError("Queue Empty");
    return false;
}

