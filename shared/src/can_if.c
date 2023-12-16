#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "board_def.h"
#include "can_if.h"
#include "log.h"

static void receive(uint8_t fifo);

static bool data_ready = false;
static canFrame msg_buffer; 

void cec_can_isr (void) {
    // Message pending on FIFO 0?
    if (CAN_RF0R(CAN1) & CAN_RF0R_FMP0_MASK) {
        receive(0);
    }

    // Message pending on FIFO 1?
    if (CAN_RF1R(CAN1) & CAN_RF1R_FMP1_MASK) {
        receive(1);
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
        CAN1,       // CAN port
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
int8_t canif_sendCanMsg (canMsg_tu* msg, uint32_t msgId) {
    int8_t res;
    if (msgId == BB_CAN_ID_HEARTBEAT) {
        res = can_transmit(CAN1, msgId, false, false, CAN_HB_SIZE, msg->au8msg);
    } else {
        res = can_transmit(CAN1, msgId, false, false, CAN_MSG_FRAME_SIZE, msg->au8msg);
    }

    return res;
}

/**
 * @brief Receive routine for CAN
 * 
 * @param fifo FIFO ID, 0 or 1
 */
static void receive(uint8_t fifo) {
    canFrame rxFrame;
    can_receive(CAN1, fifo, true, 
                &rxFrame.id, 
                &rxFrame.ext_id, 
                &rxFrame.rtr, 
                &rxFrame.filter_id, 
                &rxFrame.len, 
                rxFrame.data, 
                &rxFrame.ts);

    data_ready = true;
    msg_buffer = rxFrame;
}

/**
 * @brief Getter function for Data Ready Flag
 * 
 * @return true if Data is ready, false otherwise
 */
bool canif_getDataReady(void) {
    return data_ready;
}

/**
 * @brief Retrieve the latest data in the buffer
 * 
 * @param[out] frame pointer to can frame
 */
void canif_getData(canFrame* frame) {
    data_ready = false;
    *frame = msg_buffer;
}