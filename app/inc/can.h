#ifndef INC_PMB_CAN_H
#define INC_PMB_CAN_H

#include "common_defines.h"

#define CAN_MSG_FRAME_SIZE          (8)
#define CAN_HB_SIZE                 (1)
typedef enum {
    BB_CAN_ID_THR_1_CTRL = 0,
    BB_CAN_ID_THR_2_CTRL,
    BB_CAN_ID_ACT_CTRL,
    BB_CAN_ID_PWR_CTRL,
    BB_CAN_ID_HEARTBEAT,
    BB_CAN_ID_TEMP_SBC,
    BB_CAN_ID_STB_STAT = 11,
    BB_CAN_ID_LED_CTRL,
    BB_CAN_ID_BUTTON_STAT,
    BB_CAN_ID_THR_1_RPM = 17,
    BB_CAN_ID_THR_2_RPM,
    BB_CAN_ID_THR_1_DUTY,
    BB_CAN_ID_THR_2_DUTY,
    BB_CAN_ID_BATT_1_STAT = 23,
    BB_CAN_ID_PMB_1_STAT,
    BB_CAN_ID_BATT_2_STAT,
    BB_CAN_ID_PMB_2_STAT,
    BB_CAN_ID_PWR_STAT = 30,
    BB_CAN_ID_ERR_TAB = 32,
    BB_CAN_ID_ERR_STB,
    BB_CAN_ID_ERR_PMB_1,
    BB_CAN_ID_ERR_PMB_2,
    BB_CAN_ID_ERR_SBCCAN
} canMsgId_te;

typedef enum {
    BB_HEARTBEAT_ID_SBC = 1,
    BB_HEARTBEAT_ID_SBCCAN,
    BB_HEARTBEAT_ID_TAB,
    BB_HEARTBEAT_ID_STB,
    BB_HEARTBEAT_ID_ACT,
    BB_HEARTBEAT_ID_PMB_1,
    BB_HEARTBEAT_ID_PMB_2
} canHbId_te;

typedef struct {
    uint8_t u8HeartbeatId;
} canMsgHb_ts;

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
} canMsgHullStat_ts;

bool PMB_can_init(void);
void PMB_can_encodeCanMsgBattStat(uint16_t* battVoltage, int32_t* battCurrent);
void PMB_can_encodeCanMsgHullStat(float* hullTemp, float* hullPressure);
int8_t PMB_can_sendCanMsg(canMsgId_te msgId);

#endif // INC_PMB_CAN_H