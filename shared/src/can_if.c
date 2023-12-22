#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "board_def.h"
#include "can_if.h"
#include "cqueue.h"
#include "log.h"

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
    can_filter_id_mask_16bit_init(1, 0x0F, 0xF0, 0x0F, 0xF0, 0, true);

    /* ID 16 ~  goes to FIFO1
	 * Filter configuration:
	 * ID: 		0x00
	 * Mask:	0x00
	 * Effect:  Accept all. But since 0~15 will go into filter bank with higher priority,
	 * only 16~ will go here
	 */
    can_filter_id_mask_16bit_init(2, 0x00, 0x00, 0x00, 0x00, 1, true);
#elif defined (BOOTLOADER) 
    /* ID 0 ~ 15 goes to FIFO0 and FIFO1
	 * Filter configuration:
	 * ID: 		0b00101000
	 * Mask:	0b11111100
	 * Effect:  Accept only ID 40, 41, 42, 43
	 */
    can_filter_id_mask_16bit_init(1, 0x28, 0xFC, 0x28, 0xFC, 0, true);
    can_filter_id_mask_16bit_init(2, 0x00, 0x00, 0x00, 0x00, 1, true);
    // can_filter_id_mask_16bit_init(2, 0x28, 0xFC, 0x28, 0xFC, 1, true);

#else 
    #error "Please define firmware variant"
#endif // Variant Switch

    log_pSuccess("CAN Init Sucessful");
    return true;
};

/**
 * @brief Send can message based on the defined can format.
 * 
 * @param msg pointer to the msg to be sent
 * @param msgId can msg id
 * @return int8_t number of bytes sent, -1 for failure
 */
int8_t canif_sendCanMsg (canMsg_tu* msg, uint8_t size, uint32_t msgId) {
    int8_t res = can_transmit(CAN1, msgId, false, false, size, msg->au8msg); // Non blocking tx
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

