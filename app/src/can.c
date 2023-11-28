#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "can.h"
#include "firmware.h"
#include "log.h"

canMsgBattStat_ts canMsgBattStat = {0, 0, 0, 0, 0, 0, 0, 0};
canMsgHullStat_ts canMsgHullStat = {0, 0, 0, 0, 0, 0, 0, 0};

#if (PMB_ID % 2 == 1)
canMsgHb_ts canMsgHb = {BB_HEARTBEAT_ID_PMB_1};
#else
canMsgHb_ts canMsgHb = {BB_HEARTBEAT_ID_PMB_2};
#endif

bool PMB_can_init(void) {
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
        BX_CAN1_BASE,       // CAN port
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
    
	/* ID 0 ~ 15 goes to FIFO0
	 * Filter configuration:
	 * ID: 		0b00001111
	 * Mask:	0b11110000
	 * Effect:  Accept all IDs below 16
	 */
    can_filter_id_mask_16bit_init(1, 0x0F, 0xF0, 0x0F, 0xF0, CAN_FIFO0, true);

    /* ID 16 ~  goes to FIFO1
	 * Filter configuration:
	 * ID: 		0x00
	 * Mask:	0x00
	 * Effect:  Accept all. But since 0~15 will go into filter bank with higher priority,
	 * only 16~ will go here
	 */
    can_filter_id_mask_16bit_init(2, 0x00, 0x00, 0x00, 0x00, CAN_FIFO1, true);
    


    log_pSuccess("CAN Init Sucessful");
    return true;
};

void PMB_can_encodeCanMsgBattStat(uint16_t* battVoltage, int32_t* battCurrent) {
    canMsgBattStat.u8Reserved1 = 0xFF;
    canMsgBattStat.u8Reserved2 = 0xFF;
    canMsgBattStat.u8Reserved3 = 0xFF;
    canMsgBattStat.u8Reserved4 = 0xFF;
    
    canMsgBattStat.u8CurrentLow = (uint8_t)((uint16_t)(*battCurrent) & 0xFF);
    canMsgBattStat.u8CurrentHigh = (uint8_t)((uint16_t)(*battCurrent) >> 8);
    canMsgBattStat.u8VoltageLow = (uint8_t)((*battVoltage) & 0xFF);
    canMsgBattStat.u8VoltageHigh = (uint8_t)((*battVoltage) >> 8);
}

void PMB_can_encodeCanMsgHullStat(float* hullTemp, float* hullPressure) {
    canMsgHullStat.u8Reserved1 = 0xFF;
    canMsgHullStat.u8Reserved2 = 0xFF;
    canMsgHullStat.u8Reserved3 = 0xFF;
    canMsgHullStat.u8Reserved4 = 0xFF;

    canMsgHullStat.u8TempLow = (uint8_t)((uint16_t)(*hullTemp) & 0xFF);
    canMsgHullStat.u8TempHigh = (uint8_t)((uint16_t)(*hullTemp) >> 8);
    canMsgHullStat.u8PressureLow = (uint8_t)((uint16_t)(*hullPressure) & 0xFF);
    canMsgHullStat.u8PressureHigh = (uint8_t)((uint16_t)(*hullPressure) >> 8);
}

int8_t PMB_can_sendCanMsg(canMsgId_te msgId) {
    int8_t res;
    switch (msgId) {
        case BB_CAN_ID_HEARTBEAT:
            res = can_transmit(BX_CAN1_BASE, msgId, false, false, CAN_HB_SIZE, (uint8_t*)&canMsgHb);            
            break;  
        case BB_CAN_ID_BATT_1_STAT:
        case BB_CAN_ID_BATT_2_STAT:
            res = can_transmit(BX_CAN1_BASE, msgId, false, false, CAN_MSG_FRAME_SIZE, (uint8_t*)&canMsgBattStat);
            break;
        case BB_CAN_ID_PMB_1_STAT:
        case BB_CAN_ID_PMB_2_STAT:
            res = can_transmit(BX_CAN1_BASE, msgId, false, false, CAN_MSG_FRAME_SIZE, (uint8_t*)&canMsgHullStat);
            break;
        default:
            log_pError("Invalid CAN ID");
            res = -1;
            break;
    }

    return res;
}