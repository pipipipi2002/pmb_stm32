#!/usr/bin/env python

import can
import crcmod
import logging
import sys
import asyncio
import git
from canine import CANineBus
from time import sleep
from logging.handlers import TimedRotatingFileHandler
from defines import *

FORMATTER = logging.Formatter("%(asctime)s — %(name)s — %(levelname)s — %(message)s")

def get_console_handler():
   console_handler = logging.StreamHandler(sys.stdout)
   console_handler.setFormatter(FORMATTER)
   return console_handler

def get_file_handler():
   file_handler = TimedRotatingFileHandler(LOG_FILE, when='midnight')
   file_handler.setFormatter(FORMATTER)
   return file_handler

def get_logger(logger_name):
   logger = logging.getLogger(logger_name)
   logger.setLevel(logging.INFO)
   logger.addHandler(get_console_handler())
   logger.addHandler(get_file_handler())
   logger.propagate = False
   return logger

def computeCrc8(data:bytearray) -> bytes:
    crc8 = crcmod.mkCrcFun(poly=0x107, rev=False, initCrc=0, xorOut=0)
    result = crc8(data)
    return result.to_bytes()

def computeCrc32(data:bytearray) -> bytes:
    crc32 = crcmod.mkCrcFun(poly=0x104C11DB7, rev=True, initCrc=0, xorOut=0xFFFFFFFF)
    result = crc32(data)
    return result.to_bytes(4, "little")

def bytesAsHexString(b:bytes, rev:bool) -> str:
    if (bool):
        b = b[::-1]
    
    return "".join([f"0x{byte:02x} " for byte in b])

class PacketException(Exception):
    def __init__(self, context, message, expected, received):
        super().__init__(context)
        self.message = message
        self.expected = expected
        self.received = received
    
    def getVerbose(self):
        return f"Error: {self.message}\nExpected: {self.expected}\nReceived: {self.received}"

class Packet:
    def __init__(self, lenType:bytes=None, data:bytearray=None, crc:bytes=None):
        # data and crc is in little endian
        self.lenType = lenType
        
        if (data != None and len(data) < PACKET_DATA_SIZE):
            to_pad = PACKET_DATA_SIZE - len(data)
            self.data = data + bytearray([0xFF]*to_pad)
        else:
            self.data = data
        
        if (crc == None and not (lenType == None or data == None)):
            self.crc = self.computeCrc()
        else:
            self.crc = crc # little endian

    def __str__(self):
        return f"Packet Information\nLentype: {bytesAsHexString(self.lenType, False)}\nLength: {self.getLength()}\nType: {self.getType()}\nCRC: {hex(self.getCrc())}\nBytes: {bytesAsHexString(self.toByteStream(), False)}"

    def getLength(self) -> int:
        return (int.from_bytes(self.lenType, "little") >> 2)
    
    def getType(self) -> int:
        return (int.from_bytes(self.lenType, "little") & 0b11)
    
    def getCrc(self) -> int:
        return (int.from_bytes(self.crc, "little"))

    def computeCrc(self) -> bytes:
        data = self.lenType + self.data
        computed = computeCrc8(data)
        return computed 
    
    def toByteStream(self) -> bytearray:
        data = self.lenType + self.data + self.crc
        return data
    
    def isUtilityPacket(self, byte:int) -> bool:
        plength = (self.getLength() == PACKET_TYPE_UTILITY_SIZE)
        ptype = (self.getType() == PACKET_TYPE_UTILITY)
        pdata = (self.data[0] == byte)
        
        return True if (plength and ptype and pdata) else False
    
    def isAckPacket(self) -> bool:
        return self.isUtilityPacket(PACKET_UTILITY_ACK_DATA)
    
    def isRetxPacket(self) -> bool:
        return self.isUtilityPacket(PACKET_UTILITY_RETX_DATA)

    def isNackPacket(self) -> bool:
        return self.isUtilityPacket(PACKET_UTILITY_NACK_DATA)

    @staticmethod
    def constructLenType(length:int, type:int) -> bytes:
        lenType = (length << 2) | type
        return lenType.to_bytes(1, "little")

packetBuffer:list[Packet] = []
retxPacket:Packet = Packet(Packet.constructLenType(PACKET_LENTYPE_SIZE, PACKET_TYPE_UTILITY), PACKET_UTILITY_RETX_DATA.to_bytes())
lastTxPacket:Packet = Packet()
rxCanBuffer:bytearray = bytearray()

def sendPacket(bus: can.Bus, id, packet:Packet):
    global lastTxPacket

    # Send through CAN
    data = packet.toByteStream()

    msg0 = can.Message(arbitration_id=id, data = data[0:8], is_extended_id=False)

    try:
        bus.send(msg0)
        sleep(SLEEP_TIME)
        lastTxPacket = packet
    except can.CanError:
        log.error("Message Failed to send")

def sendData(bus:can.Bus, id:int, data:bytearray, len:int) -> int:
    frames_sent:int = 0
    i:int = 0
    while (i < len):
        if (i+8 > len):
            to_pad = (8 + i) - len
            data = data + bytearray([0xFF]*to_pad)
 
        msg = can.Message(arbitration_id=id, data=data[i:i+8], is_extended_id=False)
        
        try:
            bus.send(msg)
            sleep(SLEEP_TIME)
        except can.CanError:
            log.error("Message Failed to send")
        i += 8
        frames_sent += 1

    return frames_sent

def readFromBuffer(n: int) -> bytearray:
    removed = rxCanBuffer[0:n]
    del rxCanBuffer[0:n]
    return bytearray(removed)

async def waitForHeartbeat(reader):
    while True:
        msg = reader.get_message(timeout=5)
        if (msg is None):
            await asyncio.sleep(0.1)
            continue
        
        if (msg.arbitration_id == CAN_HEARTBEAT_ID):
            if (msg.data[0] == CAN_PMB1_HEARTBEAT_ID or msg.data[0] == CAN_PMB2_HEARTBEAT_ID):
                return True

async def waitForSingleBytePacket(queue, byte):
    packet:Packet = await queue.get()
    try:
        if ((packet.getLength() != 1) or (packet.data[0] != byte)):
            raise PacketException("Wait for Single Byte Packet", "Unexpected packet received", byte, packet)
        
        queue.task_done()
        
    except PacketException as e:
        log.error(e)
        log.error(e.getVerbose())
        exit(1)

async def waitForDataBytePacket(queue, type, word):
    packet:Packet = await queue.get()
    try:
        # Check packet length
        if (packet.getLength() != 5):
            raise PacketException("Wait for Data Byte Packet", "Wrong Length", 5, packet.getLength())
        
        # Check packet type (first data byte)
        if (packet.data[0] != type):
            raise PacketException("Wait for Data Byte Packet", "Wrong data type", type, packet.data[0])

        # Check packet data
        recvWord = int.from_bytes(packet.data[1:5], "little")
        if (recvWord != word):
            raise PacketException("Wait for Data Byte Packet", "Wrong data word", word, recvWord)
        
        queue.task_done()
        
    except PacketException as e:
        log.error(e)
        log.error(e.getVerbose())
        exit(1)
            
async def packetBuilder(bus, reader, queue):
    global rxCanBuffer
    while True:
        msg = reader.get_message()
        if (msg is not None):
            # Process the message to build a packet
            if (msg.arbitration_id == CAN_BOOTLOADER_ID):
                log.debug("Received CAN from PMB")
                rxCanBuffer = rxCanBuffer + msg.data
                
                # Try to build a Packet
                if (len(rxCanBuffer) >= PACKET_TOTAL_SIZE):
                    log.debug("Buidling packet")
                    # Extract data from buffer
                    rawBytes: bytearray = readFromBuffer(PACKET_TOTAL_SIZE) 

                    lenType = rawBytes[0].to_bytes()
                    data = rawBytes[PACKET_LENTYPE_SIZE:PACKET_DATA_SIZE+PACKET_LENTYPE_SIZE]
                    crc = rawBytes[PACKET_DATA_SIZE+PACKET_LENTYPE_SIZE:]

                    packet = Packet(lenType, data, crc)
                    computedCrc = packet.computeCrc()

                    # Need Retransmission
                    if (computedCrc != packet.crc):
                        log.error(f"Wrong CRC! Received: {packet.getCrc()}, Computed: {int.from_bytes(computedCrc, 'little')}")
                        log.debug("Sending ReTx Packet")
                        sendPacket(bus, CAN_BOOTLOADER_ID, retxPacket) 
                        continue

                    # Retransmission packet
                    if (packet.isRetxPacket()):
                        log.debug("Received ReTx Packet, Retransmitting last packet")
                        sendPacket(bus, CAN_BOOTLOADER_ID, lastTxPacket)
                        continue
                    
                    # NACK packet
                    if (packet.isNackPacket()):
                        log.error("Received NACK Packet")
                        exit(1)

                    log.debug("Received normal packet")
                    # Append packet to buffer
                    await queue.put(packet) # push to queue

                    continue

            else:   
                log.debug("Received invalid ID")

        await asyncio.sleep(0.1)
    
async def main(log: logging.Logger):
    log.info(f"Reading Firmware Image {FW_FILE}")
    with open(FW_FILE, "rb") as f:
        bin_file = f.read()
        f.close()

    fw_bin_main_app:bytearray = bytearray(bin_file[BOOTLOADER_SIZE+METADATA_SIZE:])
    log.info("Removed bootloader partition")

    fw_device_id:int = int.from_bytes(fw_bin_main_app[FIRMWARE_INFO_OFFSET:FIRMWARE_INFO_OFFSET+4], "little")
    log.info(f"Firmare device id: {hex(fw_device_id)}")

    fw_length:int = len(fw_bin_main_app)    
    log.info(f"Firmware Length: {fw_length}")

    repo = git.Repo(search_parent_directories=True)
    commit_sha = repo.head.object.hexsha
    fw_version:int = int(commit_sha[:8], 16)
    log.info(f"Firmware version: {hex(fw_version)}")

    fw_version_ba:bytearray = bytearray(fw_version.to_bytes(4, "little"))
    for i in range(4):
        fw_bin_main_app[FIRMWARE_INFO_COMMIT_OFFSET+i] = fw_version_ba[i]
    log.info("Inject version to Firmware")

    fw_crc32_le:bytes = computeCrc32(fw_bin_main_app)
    fw_crc = int.from_bytes(fw_crc32_le, "little")
    log.info(f"Firmware CRC-32: {hex(fw_crc)}")

    with can.Bus(interface='canine', bitrate=1000000) as bus:
        reader = can.BufferedReader()
        packet_queue = asyncio.Queue()
        notifier = can.Notifier(bus, [reader])

        # Wait for heartbeat from Bootloader 
        log.info("Waiting for heartbeat")
        await waitForHeartbeat(reader)

        # Start packet building task
        packetBuilder_task = asyncio.create_task(packetBuilder(bus, reader, packet_queue))
        
        # Send Firmware Update Request and wait for ACK
        furPacket = Packet(Packet.constructLenType(1, PACKET_TYPE_NORMAL), BL_FUR_REQ_PACKET.to_bytes())
        log.info(f"Sending Firmware Update Request Packet: {bytesAsHexString(BL_FUR_REQ_PACKET.to_bytes(), False)}")
        sendPacket(bus, CAN_BOOTLOADER_ID, furPacket)
        log.info("Waiting for Firmware Update Request Ack Packet")
        await waitForSingleBytePacket(packet_queue, BL_FUR_ACK_RES_PACKET)
        log.warning("Received Firmware Update Request Ack Packet")

        # Send Device ID and wait for ACK
        devIdPayload = bytearray(BL_DEVID_REQ_PACKET.to_bytes()) + bytearray(fw_device_id.to_bytes(4, "little"))
        devIdPacket = Packet(Packet.constructLenType(5, PACKET_TYPE_NORMAL), devIdPayload)
        log.info(f"Sending Device Id Packet: {bytesAsHexString(devIdPayload, False)}")
        sendPacket(bus, CAN_BOOTLOADER_ID, devIdPacket)
        log.info("Waiting for Device ID Ack Packet")
        await waitForDataBytePacket(packet_queue, BL_DEVID_ACK_RES_PACKET, fw_device_id)
        log.warning("Received Device ID ACK Packet")
        
        # Send Firmware Size and wait for ACK
        fwLenPayload = bytearray(BL_FWLEN_REQ_PACKET.to_bytes()) + bytearray(fw_length.to_bytes(4, "little"))
        fwLenPacket = Packet(Packet.constructLenType(5, PACKET_TYPE_NORMAL), fwLenPayload)
        log.info(f"Sending Firmware Length Packet: {fw_length}")
        sendPacket(bus, CAN_BOOTLOADER_ID, fwLenPacket)
        log.info("Waiting for Firmware Length Ack Packet")
        await waitForDataBytePacket(packet_queue, BL_FWLEN_ACK_RES_PACKET, fw_length)
        log.warning("Received Firmware Length Ack Packet")

        # Send CRC and wait for ACK
        fwCRCPayload = bytearray(BL_FWCRC_REQ_PACKET.to_bytes()) + bytearray(fw_crc.to_bytes(4, "little"))
        fwCRCPacket = Packet(Packet.constructLenType(5, PACKET_TYPE_NORMAL), fwCRCPayload)
        log.info(f"Sending Firmware CRC Packet: {hex(fw_crc)}")
        sendPacket(bus, CAN_BOOTLOADER_ID, fwCRCPacket)
        log.info("Waiting for Firmware CRC Ack Packet")
        await waitForDataBytePacket(packet_queue, BL_FWCRC_ACK_RES_PACKET, fw_crc)
        log.warning("Received Firmware CRC Ack Packet")

        # Send Version and wait for ACK
        fwVerPayload = bytearray(BL_FWVER_REQ_PACKET.to_bytes()) + bytearray(fw_version.to_bytes(4, "little"))
        fwVerPacket = Packet(Packet.constructLenType(5, PACKET_TYPE_NORMAL), fwVerPayload)
        log.info(f"Sending Firmware Version Packet: {hex(fw_version)}")
        sendPacket(bus, CAN_BOOTLOADER_ID, fwVerPacket)
        log.info("Waiting for Firmware Version Ack Packet")
        await waitForDataBytePacket(packet_queue, BL_FWVER_ACK_RES_PACKET, fw_version)
        log.warning("Received Firmware Version Ack Packet")
        
        # (wait for Data Ready) Start Firmware Update
        bytesSent: int = 0    
        while (bytesSent < fw_length):
            # Wait for data ready
            log.info("Waiting for Receive Ready Packet")
            await waitForSingleBytePacket(packet_queue, BL_RECVRDY_RES_PACKET)
            log.warning("Received Receive Ready Packet")

            # Check length to send
            lenToSend = BL_FW_DATA_SEG_SIZE if (bytesSent +  BL_FW_DATA_SEG_SIZE <= fw_length) else (fw_length - bytesSent)
            fwDataSegment = fw_bin_main_app[bytesSent: bytesSent + lenToSend]
            
            # Send FW data to DATA ID
            log.info("Sending data over...")
            frameSent = sendData(bus, CAN_BOOTLOADER_DATA_ID, fwDataSegment, lenToSend)
            log.warning(f"[{lenToSend+bytesSent}/{fw_length}] Sent {lenToSend} bytes over {frameSent} CAN frames")

            # Send FW data sent packet
            fwDataSentPayload = bytearray(BL_DATASENT_REQ_PACKET.to_bytes()) + bytearray(lenToSend.to_bytes(4, "little"))
            fwDataSentPacket = Packet(Packet.constructLenType(5, PACKET_TYPE_NORMAL), fwDataSentPayload)
            log.info("Sending FW Data Sent Packet")
            sendPacket(bus, CAN_BOOTLOADER_ID, fwDataSentPacket)

            bytesSent += lenToSend

        # wait for success
        log.info("Waiting for Update Success Packet")
        await waitForSingleBytePacket(packet_queue, BL_SUCCESS_RES_PACKET)
        log.warning("Received Success Update Packet")

if __name__ == "__main__":
    log = get_logger("fw_log")
    log.info("Start FW Updator")

    asyncio.run(main(log))