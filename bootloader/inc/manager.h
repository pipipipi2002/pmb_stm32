#ifndef INC_MANAGER_H
#define INC_MANAGER_H

#include "common_defines.h"

#define PACKET_BUFFER_LENGTH        (8U)

#define PACKET_LENTYPE_SIZE         (1U)
#define PACKET_DATA_SIZE            (19U)
#define PACKET_CRC_SIZE             (4U)
#define PACKET_TOTAL_SIZE           (PACKET_LENTYPE_SIZE + PACKET_DATA_SIZE + PACKET_CRC_SIZE)

#define PACKET_LENGTH_MASK          (0b11111100)
#define PACKET_TYPE_MASK            (0b00000011)
#define PACKET_TYPE_NORMAL_DATA     (1U)
#define PACKET_TYPE_UTILITY         (2U)
#define PACKET_CONSTRUCT_LENGTHTYPE(len, type) ((uint8_t) (len << 2) | type)

#define PACKET_UTILITY_ACK_DATA     (0x35)
#define PACKET_UTILITY_RETX_DATA    (0x67)
#define PACKET_UTILITY_NACK_DATA    (0xDE)

typedef struct {
    uint8_t lenType;
    uint8_t data[PACKET_DATA_SIZE];
    uint32_t crc;
} man_packet_ts;

void man_setup(void);
void man_update(void);

void man_sendNack(void);
void man_write(man_packet_ts* packet);
void man_read(man_packet_ts* packet);
bool man_packetAvailable(void);
bool man_packetBufferFull(void);
void man_createBLPacketSingle(man_packet_ts* packet, uint8_t type);
bool man_isBLPacketSingle(const man_packet_ts* packet, const uint8_t type);
void man_createBLPacketData(man_packet_ts* packet, uint8_t type, uint32_t data);
bool man_isBLPacketData(const man_packet_ts* packet, const uint8_t type, const uint32_t dataCompare);

#endif // INC_MANAGER_H