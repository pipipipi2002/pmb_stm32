#include "manager.h"
#include "can_if.h"
#include "crc_if.h"
#include "log.h"
#include "bootloader_defines.h"

typedef enum {
    MAN_STATE_LENGTHTYPE = 0,
    MAN_STATE_DATA,
    MAN_STATE_CRC
} man_state_te;

static man_state_te state = MAN_STATE_LENGTHTYPE;
static uint8_t dataBytesReceived = 0;

static man_packet_ts tempPacket = {.lenType = 0, .data = {0}, .crc = 0};
static man_packet_ts retxPacket = {.lenType = 0, .data = {0}, .crc = 0};
static man_packet_ts nackPacket = {.lenType = 0, .data = {0}, .crc = 0};
static man_packet_ts lastTxPacket = {.lenType = 0, .data = {0}, .crc = 0};

/* Packet Recv Circular Buffer */
static man_packet_ts packetBuffer[PACKET_BUFFER_LENGTH];
static uint8_t packetWriteIndex = 0;
static uint8_t packetReadIndex = 0;
static uint8_t packetBufferLength = 0;

static void createSingleBytePacket(man_packet_ts* packet, uint8_t data, uint8_t packet_type);
static bool isUtilityPacket(const man_packet_ts* packet, uint8_t data);
static void createUtilityPacket(man_packet_ts* packet, uint8_t data);

/**
 * @brief Create ack and retx packet which will always be the same
 *          Called once only.
 * 
 */
void man_setup(void) {
    createUtilityPacket(&retxPacket, PACKET_UTILITY_RETX_DATA);
    createUtilityPacket(&nackPacket, PACKET_UTILITY_NACK_DATA);
}

/**
 * @brief Main state machine to handle packet. Needs to be called periodically.
 * 
 */
void man_update(void) {
    while (canif_getRxDataReady()) {
        switch(state) {
            case MAN_STATE_LENGTHTYPE: {
                /* Get Length+Type Byte */
                canif_getRxData(CAN_ID_BOOTLOADER, &(tempPacket.lenType));
                $INFO("Received LenType");
                state = MAN_STATE_DATA;
            } break;

            case MAN_STATE_DATA: {
                /* Get Data Bytes */
                canif_getRxData(CAN_ID_BOOTLOADER, &(tempPacket.data[dataBytesReceived++]));
                
                /* Check if data received == length */
                if (dataBytesReceived >= PACKET_DATA_SIZE) {
                    dataBytesReceived = 0;
                    state = MAN_STATE_CRC;
                    $INFO("Received Data");
                }
            } break;

            case MAN_STATE_CRC: {
                /* Get CRC Bytes */
                canif_getRxData(CAN_ID_BOOTLOADER, &(tempPacket.crc));
                $INFO("Received CRC");
                
                /* Compute CRC-8 */
                uint8_t computedCrc = crcif_compute8((uint8_t*)&tempPacket, (PACKET_TOTAL_SIZE - PACKET_CRC_SIZE));
 
                /* Check CRC */
                if (computedCrc != tempPacket.crc) {
                    /* Request Retransmission */
                    $ERROR("CRC mismatch, recv: 0x%X, computed: 0x%X. Send ReTx Packet", tempPacket.crc, computedCrc);
                    man_write(&retxPacket);
                    state = MAN_STATE_LENGTHTYPE;
                    break;
                }
                $INFO("CRC Match");

                /* Handle Packet based on its Type */
                uint8_t packetType = (uint8_t)(tempPacket.lenType & PACKET_TYPE_MASK);
                switch (packetType) {
                    case PACKET_TYPE_NORMAL_DATA: {
                        /* Check buffer full */
                        if (man_packetBufferFull()) {
                            $ERROR("Packet Buffer Full, dropping packet");
                            break;
                        }
                        $SUCCESS("Received Normal Packet");

                        /* Write into buffer */
                        memcpy(&(packetBuffer[packetWriteIndex++]), &tempPacket, sizeof(man_packet_ts));
                        packetWriteIndex %= PACKET_BUFFER_LENGTH;
                        packetBufferLength++;

                    } break;
                    
                    case PACKET_TYPE_UTILITY: {
                        /* Check Utility Type */
                        if (isUtilityPacket(&tempPacket, PACKET_UTILITY_RETX_DATA)) { // Retx
                            $INFO("Received Retx, send last tx");
                            man_write(&lastTxPacket);
                        } else {
                            $ERROR("Received unknown utility data: %X", tempPacket.data[0]); // Undefined
                        }
                    } break;
                    
                    default: {
                        $ERROR("Received Wrong Packet Type: %d", packetType);
                    } break;
                }
                /* Reset to beginning */
                state = MAN_STATE_LENGTHTYPE;
            } break;

            default: {
                $ERROR("Entered Undefined Manager State");
                state = MAN_STATE_LENGTHTYPE;
            }  break;
        }
    }
}

void man_sendNack(void) {
    man_write(&nackPacket);
}

/**
 * @brief Send packet through can line and store it in the transmit buffer 
 * 
 * @param packet pointer to the packet
 */
void man_write(man_packet_ts* packet) {
    /* Send data in CAN */
    canif_sendData((uint8_t*)packet, PACKET_TOTAL_SIZE, CAN_ID_BOOTLOADER);
    /* Store to last transmitted buffer */
    memcpy(&lastTxPacket, packet, sizeof(man_packet_ts));

}

/**
 * @brief Read and pop packet from the packet recv buffer. 
 * 
 * @param packet pointer to the where the packet will be stored 
 */
void man_read(man_packet_ts* packet) {
    memcpy(packet, &(packetBuffer[packetReadIndex++]), sizeof(man_packet_ts));
    packetReadIndex %= PACKET_BUFFER_LENGTH;
    packetBufferLength--;
}

/**
 * @brief Check whether packet recv buffer is not empty
 * 
 * @return true if exists packet
 */
bool man_packetAvailable(void) {
    return (packetBufferLength > 0);
}

/**
 * @brief Check whther packet recv buffer is full
 * 
 * @return true 
 * @return false 
 */
bool man_packetBufferFull(void) {
    return (packetBufferLength == PACKET_BUFFER_LENGTH);
}

/**
 * @brief Create Single Byte Normal Packet with 1 byte payload
 * 
 * @param packet Pointer to packet
 * @param type DATA0 byte
 */
void man_createBLPacketSingle(man_packet_ts* packet, uint8_t type) {
    createSingleBytePacket(packet, type, PACKET_TYPE_NORMAL_DATA);
}

/**
 * @brief Check Singel Byte Normal Packet with 1 byte payload and verify payload
 * 
 * @param packet pointer to packet
 * @param type type expected
 * @return true if packet DATA0 == type
 */
bool man_isBLPacketSingle(const man_packet_ts* packet, const uint8_t type) {
    /* Check packet is a byte */
    if (packet->lenType >> 2 != 1) {
        return false;
    }
    /* Check type */
    if (packet->data[0] != type) {
        return false;
    }
    return true;
}

/**
 * @brief Create 5 Byte Payload Normal Packet. DATA0: Type, DATA1,2,3,4: data
 * 
 * @param packet Pointer to packet
 * @param type Payload type
 * @param data Payload data
 */
void man_createBLPacketData(man_packet_ts* packet, uint8_t type, uint32_t data) {
    memset(packet, 0xFF, sizeof(man_packet_ts));
    packet->lenType = PACKET_CONSTRUCT_LENGTHTYPE(5, PACKET_TYPE_NORMAL_DATA);
    packet->data[0] = type;
    packet->data[1] = data & 0xFF;
    packet->data[2] = (data >> 8) & 0xFF;
    packet->data[3] = (data >> 16) & 0xFF;
    packet->data[4] = (data >> 24) & 0xFF;
    packet->crc = crcif_compute8((uint8_t *) packet, (PACKET_TOTAL_SIZE - PACKET_CRC_SIZE));
}

/**
 * @brief Check 5 Byte Payload Normal Packet. 
 * 
 * @param packet Pointer to packet
 * @param type Payload type expected
 * @param dataCompare Payload data expected
 * @return true if type and data is expected
 */
bool man_isBLPacketData(const man_packet_ts* packet, const uint8_t type, const uint32_t dataCompare) {
    /* Check Length */
    if (packet->lenType >> 2 != 5) {
        return false;
    }
    /* Check Type */
    if (packet->data[0] != type) {
        return false;
    }
    /* Check data */
    if (dataCompare != 0) {
        uint32_t data = packet->data[1] | (packet->data[2] << 8) | (packet->data[3] << 16) | (packet->data[4] << 24);
        if (dataCompare != data) {
            return false;
        }
    }
    return true;
}


/**
 * @brief Check packet is a utility packet and equiv to the packet data
 * 
 * @param packet pointer to the packet
 * @param data data byte of the utility data
 * @return true if matches
 */
static bool isUtilityPacket(const man_packet_ts* packet, const uint8_t data) {
    if (packet->lenType != 0b00000110) {
        return false;
    }
    if (packet->data[0] != data) {
        return false;
    }
    for (uint8_t i = 1; i < PACKET_DATA_SIZE; i++) {
        if (packet->data[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Create a Single Byte Packet 
 * 
 * @param packet Pointer to the packet
 * @param data Data0 
 * @param packet_type NORMAL OR UTILITY 
 */
static void createSingleBytePacket(man_packet_ts* packet, uint8_t data, uint8_t packet_type) {
    memset(packet, 0xFF, sizeof(man_packet_ts));
    packet->lenType = PACKET_CONSTRUCT_LENGTHTYPE(1, packet_type);
    packet->data[0] = data;
    packet->crc = crcif_compute8((uint8_t *) packet, (PACKET_TOTAL_SIZE - PACKET_CRC_SIZE));
}

/**
 * @brief Create a Utility Packet object
 * 
 * @param packet Pointer to the packet
 * @param data Utility Type
 */
static void createUtilityPacket(man_packet_ts* packet, uint8_t data) {
    createSingleBytePacket(packet, data, PACKET_TYPE_UTILITY);
}