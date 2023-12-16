#ifndef INC_PMB_CAN_IF_H
#define INC_PMB_CAN_IF_H

#include "common_defines.h"
#include "can_defines.h"

typedef struct {
    uint32_t id;
    bool ext_id;
    bool rtr;
    uint8_t filter_id;
    uint8_t len;
    uint8_t data[8];
    uint16_t ts;
} canFrame_ts;

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
int8_t canif_sendCanMsg (canMsg_tu* msg, uint8_t size, uint32_t msgId);
bool canif_getRxDataReady(void);
void canif_getRxData(canFrame_ts* frame);

#endif // INC_PMB_CAN_IF_H