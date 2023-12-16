#ifndef INC_PMB_CAN_IF_H
#define INC_PMB_CAN_IF_H

#include "common_defines.h"

#define CAN_MSG_FRAME_SIZE                  (8)
#define CAN_HB_SIZE                         (1)

/*
 * CAN Message IDs
 */
#define BB_CAN_ID_THR_1_CTRL                (0)
#define BB_CAN_ID_THR_2_CTRL                (1)
#define BB_CAN_ID_ACT_CTRL                  (2)
#define BB_CAN_ID_PWR_CTRL                  (3)
#define BB_CAN_ID_HEARTBEAT                 (4)
#define BB_CAN_ID_TEMP_SBC                  (5)

#define BB_CAN_ID_STB_STAT                  (11)
#define BB_CAN_ID_LED_CTRL                  (12)
#define BB_CAN_ID_BUTTON_STAT               (13)

#define BB_CAN_ID_THR_1_RPM                 (17)
#define BB_CAN_ID_THR_2_RPM                 (18)
#define BB_CAN_ID_THR_1_DUTY                (19)
#define BB_CAN_ID_THR_2_DUTY                (20)

#define BB_CAN_ID_BATT_1_STAT               (23)
#define BB_CAN_ID_PMB_1_STAT                (24)
#define BB_CAN_ID_BATT_2_STAT               (25)
#define BB_CAN_ID_PMB_2_STAT                (26)

#define BB_CAN_ID_PWR_STAT                  (30)

#define BB_CAN_ID_ERR_TAB                   (32)
#define BB_CAN_ID_ERR_STB                   (33)
#define BB_CAN_ID_ERR_PMB_1                 (34)
#define BB_CAN_ID_ERR_PMB_2                 (35)
#define BB_CAN_ID_ERR_SBCCAN                (36)

/*
 * Heartbeat IDs
 */
#define BB_HEARTBEAT_ID_SBC                 (1)
#define BB_HEARTBEAT_ID_SBCCAN              (2)
#define BB_HEARTBEAT_ID_TAB                 (4)
#define BB_HEARTBEAT_ID_STB                 (5)
#define BB_HEARTBEAT_ID_ACT                 (6)
#define BB_HEARTBEAT_ID_PMB_1               (7)
#define BB_HEARTBEAT_ID_PMB_2               (8)

typedef struct {
    uint32_t id;
    bool ext_id;
    bool rtr;
    uint8_t filter_id;
    uint8_t len;
    uint8_t data[8];
    uint16_t ts;
} canFrame;

/*
 * Semantic definition of the CAN message frame
 */
typedef struct  {
    uint8_t u8CurrentLow;
    uint8_t u8CurrentHigh;
    uint8_t u8VoltageLow;
    uint8_t u8VoltageHigh;
    uint8_t u8Reserved1;
    uint8_t u8Reserved2;
    uint8_t u8Reserved3;
    uint8_t u8Reserved4;
} canMsgBattStat_ts;

typedef struct  {
    uint8_t u8TempLow;
    uint8_t u8TempHigh;
    uint8_t u8PressureLow;
    uint8_t u8PressureHigh;
    uint8_t u8Reserved1;
    uint8_t u8Reserved2;
    uint8_t u8Reserved3;
    uint8_t u8Reserved4;
} canMsgBoardStat_ts;

typedef union {
    canMsgBattStat_ts battStat;
    canMsgBoardStat_ts boardStat;
    uint8_t heartbeatId;
    uint8_t au8msg[8];
} canMsg_tu;

/*
 * Public facing interface
 */
bool canif_setup(void);
int8_t canif_sendCanMsg (canMsg_tu* msg, uint32_t msgId);
bool canif_getDataReady(void);
void canif_getData(canFrame* frame);

#endif // INC_PMB_CAN_IF_H