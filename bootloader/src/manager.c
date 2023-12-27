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
static uint8_t crcBytesReceived = 0;

static man_packet_ts tempPacket = {.lenType = 0, .data = {0}, .crc = 0};
static man_packet_ts retxPacket = {.lenType = 0, .data = {0}, .crc = 0};
static man_packet_ts ackPacket = {.lenType = 0, .data = {0}, .crc = 0};
static man_packet_ts lastTxPacket = {.lenType = 0, .data = {0}, .crc = 0};

/* Packet Recv Circular Buffer */
static man_packet_ts packetBuffer[PACKET_BUFFER_LENGTH];
static uint8_t recvCrc[4];
static uint8_t packetWriteIndex = 0;
static uint8_t packetReadIndex = 0;
static uint8_t packetBufferLength = 0;

static bool isUtilityPacket(const man_packet_ts* packet, uint8_t data);
static void createUtilityPacket(man_packet_ts* packet, uint8_t data);
static void copyPacket(const man_packet_ts* src, man_packet_ts* dest);

/**
 * @brief Create ack and retx packet which will always be the same
 *          Called once only.
 * 
 */
void man_setup(void) {
    createUtilityPacket(&retxPacket, PACKET_UTILITY_RETX_DATA);
    createUtilityPacket(&ackPacket, PACKET_UTILITY_ACK_DATA);
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
                canif_getRxData(&(tempPacket.lenType));
                log_pInfo("Received LenType");
                state = MAN_STATE_DATA;
            } break;

            case MAN_STATE_DATA: {
                /* Get Data Bytes */
                canif_getRxData(&(tempPacket.data[dataBytesReceived++]));
                
                /* Check if data received == length */
                if (dataBytesReceived >= PACKET_DATA_SIZE) {
                    dataBytesReceived = 0;
                    state = MAN_STATE_CRC;
                    log_pInfo("Received Data");
                }
            } break;

            case MAN_STATE_CRC: {
                /* Get CRC Bytes */
                canif_getRxData(&(recvCrc[crcBytesReceived++]));

                if (crcBytesReceived >= PACKET_CRC_SIZE) {
                    crcBytesReceived = 0;
                    log_pInfo("Received CRC");
                } else {
                    /* Continue receiving CRC bytes */
                    break;
                }

                /* Construct CRC from the bytes received */
                tempPacket.crc = recvCrc[0] | (recvCrc[1] << 8) | (recvCrc[2] << 16) | (recvCrc[3] << 24); // Little endian
                
                /* Compute CRC-32 */
                uint32_t computedCrc = crcif_compute32((uint8_t*)&tempPacket, (PACKET_TOTAL_SIZE - PACKET_CRC_SIZE));
 
                /* Check CRC */
                if (computedCrc != tempPacket.crc) {
                    /* Request Retransmission */
                    log_pError("CRC mismatch, recv: 0x%X, computed: 0x%X. Send ReTx Packet", tempPacket.crc, computedCrc);
                    man_write(&retxPacket);
                    state = MAN_STATE_LENGTHTYPE;
                    break;
                }
                log_pInfo ("CRC Match");

                /* Handle Packet based on its Type */
                uint8_t packetType = (uint8_t)(tempPacket.lenType & PACKET_TYPE_MASK);
                switch (packetType) {
                    case PACKET_TYPE_NORMAL_DATA: {
                        /* Check buffer full */
                        if (man_packetBufferFull()) {
                            log_pError("Packet Buffer Full, dropping packet");
                            break;
                        }
                        log_pSuccess("Received Normal Packet");

                        /* Write into buffer */
                        copyPacket(&tempPacket, &(packetBuffer[packetWriteIndex++]));
                        packetWriteIndex %= PACKET_BUFFER_LENGTH;
                        packetBufferLength++;

                        /* Send acknowledge packet */
                        man_write(&ackPacket);
                        log_pInfo("Send ACK packet");

                    } break;
                    
                    case PACKET_TYPE_UTILITY: {
                        /* Check Utility Type */
                        if (isUtilityPacket(&tempPacket, PACKET_UTILITY_RETX_DATA)) { // Retx
                            log_pInfo("Received Retx, send last tx");
                            man_write(&lastTxPacket);
                        } else if (isUtilityPacket(&tempPacket, PACKET_UTILITY_ACK_DATA)) { // ACK
                            log_pInfo("Received ACK");
                            break;
                        } else {
                            log_pError("Received unknown utility data: %X", tempPacket.data[0]); // Undefined
                        }
                    } break;
                    
                    default: {
                        log_pError("Received Wrong Packet Type: %d", packetType);
                    } break;
                }
                /* Reset to beginning */
                state = MAN_STATE_LENGTHTYPE;
            } break;

            default: {
                log_pError("Entered Undefined Manager State");
                state = MAN_STATE_LENGTHTYPE;
            }  break;
        }
    }
}


/**
 * @brief Send packet through can line and store it in the transmit buffer 
 * 
 * @param packet pointer to the packet
 */
void man_write(man_packet_ts* packet) {
    /* Send data in CAN */
    canif_sendData((uint8_t*)packet, PACKET_TOTAL_SIZE, CAN_ID_BOOTLOADER_PMB);
    /* Store to last transmitted buffer */
    copyPacket(packet, &lastTxPacket);
}

/**
 * @brief Read and pop packet from the packet recv buffer. 
 * 
 * @param packet pointer to the where the packet will be stored 
 */
void man_read(man_packet_ts* packet) {
    copyPacket(&packetBuffer[packetReadIndex++], packet);
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

static void createUtilityPacket(man_packet_ts* packet, uint8_t data) {
    packet->lenType = PACKET_CONSTRUCT_LENGTHTYPE(1, PACKET_TYPE_UTILITY);
    packet->data[0] = data;
    for (uint8_t i = 1; i < PACKET_DATA_SIZE-1; i++) {
        packet->data[i] = 0xFF;
    }
    packet->crc = crcif_compute32((uint8_t *) packet, (PACKET_TOTAL_SIZE - PACKET_CRC_SIZE));
}

static void copyPacket(const man_packet_ts* src, man_packet_ts* dest) {
    dest->lenType = src->lenType;
    for (uint8_t i = 0; i < PACKET_DATA_SIZE; i++) {
        dest->data[i] = src->data[i];
    }
    dest->crc = src->crc;
}