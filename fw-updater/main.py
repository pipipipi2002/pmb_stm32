#!/usr/bin/env python

import can
import crcmod
import logging
import sys
from canine import CANineBus
from time import sleep
from logging.handlers import TimedRotatingFileHandler
from can.notifier import MessageRecipient
import asyncio
from signal import signal, SIGINT
from sys import exit

from asyncio import Future

CAN_BOOTLOADER_SERVER_ID = 40
CAN_BOOTLOADER_PMB_ID = 41

CMD_JUMP = 0xAA
CMD_ECHO = 0xBB

PACKET_LENTYPE_SIZE = 1
PACKET_DATA_SIZE = 19
PACKET_CRC_SIZE = 4
PACKET_TOTAL_SIZE = PACKET_LENTYPE_SIZE + PACKET_DATA_SIZE + PACKET_CRC_SIZE 

PACKET_TYPE_NORMAL = 1
PACKET_TYPE_UTILITY = 2
PACKET_TYPE_UTILITY_SIZE = 1

PACKET_UTILITY_ACK_DATA = 0xAA
PACKET_UTILITY_RETX_DATA = 0x55

SLEEP_TIME = 0.1 # interval between can frames

FORMATTER = logging.Formatter("%(asctime)s — %(name)s — %(levelname)s — %(message)s")
LOG_FILE = "fw-updater.log"

shutdown = False

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
   logger.setLevel(logging.DEBUG)
   logger.addHandler(get_console_handler())
   logger.addHandler(get_file_handler())
   logger.propagate = False
   return logger

def computeCrc32(data:bytearray) -> bytes:
    crc32 = crcmod.mkCrcFun(poly=0x104C11DB7, rev=True, initCrc=0, xorOut=0xFFFFFFFF)
    result = crc32(data)
    return result.to_bytes(4, "little")

def bytesAsHexString(b:bytes) -> str:
    return "".join([f"0x{byte:02x} " for byte in b])

class Packet:
    def __init__(self, lenType:bytes=None, data:bytearray=None, crc:bytearray=None):
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
        return f"Packet Information\nLentype: {bytesAsHexString(self.lenType)}\nLength: {self.getLength()}\nType: {self.getType()}\nCRC: {hex(self.getCrc())}\nBytes: {bytesAsHexString(self.toByteStream())}"

    def getLength(self) -> int:
        return (int.from_bytes(self.lenType, "little") >> 2)
    
    def getType(self) -> int:
        return (int.from_bytes(self.lenType, "little") & 0b11)
    
    def getCrc(self) -> int:
        return (int.from_bytes(self.crc, "little"))

    def computeCrc(self) -> bytearray:
        data = self.lenType + self.data
        computed = bytearray(computeCrc32(data)) # Little endian already
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

    @staticmethod
    def constructLenType(length:int, type:int) -> bytes:
        lenType = (length << 2) | type
        return lenType.to_bytes(1, "little")

def send(opt):
    with can.Bus(interface='canine', bitrate=1000000) as bus:

        if opt == 1: # Echo
            data_to_send = [CMD_JUMP]
        elif opt == 2:
            data_to_send = [CMD_ECHO]
        elif opt == 3:
            data_to_send = [0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89]
        elif opt == 4:
            data_to_send = [0x00] * 8
        
        send_data(CAN_BOOTLOADER_SERVER_ID, data_to_send)

def send_data(id, data):
    with can.Bus(interface='canine', bitrate=1000000) as bus:
        msg = can.Message(
            arbitration_id=id, data=data, is_extended_id=False
        )

        try:
            bus.send(msg)
            print(f"Message sent to {CAN_BOOTLOADER_SERVER_ID}")
        except can.CanError:
            print("Message NOT sent")

def start_rx():
    with can.Bus(interface='canine', bitrate=1000000) as bus:
        try:
            for msg in bus:
                print(msg.arbitration_id)
                print(msg.data)
                return
        except:
            print("error")


packetBuffer:list[Packet] = []

ackPacket:Packet = Packet(Packet.constructLenType(PACKET_LENTYPE_SIZE, PACKET_TYPE_UTILITY), PACKET_UTILITY_ACK_DATA.to_bytes())
retxPacket:Packet = Packet(Packet.constructLenType(PACKET_LENTYPE_SIZE, PACKET_TYPE_UTILITY), PACKET_UTILITY_RETX_DATA.to_bytes())

lastTxPacket:Packet = Packet()

rxCanBuffer:bytearray = bytearray()

def sendPacket(bus: can.Bus, id, packet:Packet):
    global lastTxPacket

    # Send through CAN
    data = packet.toByteStream()

    msg0 = can.Message(arbitration_id=id, data = data[0:8], is_extended_id=False)
    msg1 = can.Message(arbitration_id=id, data = data[8:16], is_extended_id=False)
    msg2 = can.Message(arbitration_id=id, data = data[16:24], is_extended_id=False)

    try:
        bus.send(msg0)
        sleep(SLEEP_TIME)
        bus.send(msg1)
        sleep(SLEEP_TIME)
        bus.send(msg2)
        sleep(SLEEP_TIME)
        lastTxPacket = packet
    except can.CanError:
        log.error("Message Failed to send")


def readFromBuffer(n: int) -> bytearray:
    removed = rxCanBuffer[0:n]
    del rxCanBuffer[0:n]
    return bytearray(removed)


# async def packetConstructor(bus, reader):
#     while (True):
#         try: 
#         # wait for reader to have message
#             msg = await asyncio.wait_for(reader.get_message(), timeout=1.0)

#             if (msg.arbitration_id == CAN_BOOTLOADER_PMB_ID):
#                 log.debug("Received CAN from PMB")
#                 rxCanBuffer = rxCanBuffer + msg.data
                
#                 # Try to build a Packet
#                 if (len(rxCanBuffer) >= PACKET_TOTAL_SIZE):
#                     # Extract data from buffer
#                     rawBytes: bytearray = readFromBuffer(PACKET_TOTAL_SIZE) 

#                     lenType = bytes(rawBytes[0])
#                     data = rawBytes[1:17]
#                     crc = rawBytes[17:]

#                     packet = Packet(lenType, data, crc)
#                     computedCrc = packet.computeCrc()

#                     # Need Retransmission
#                     if (computedCrc != packet.crc):
#                         log.error(f"Wrong CRC! Received: {packet.getCrc()}, Computed: {int.from_bytes(computedCrc, "little")}")
#                         log.debug("Sending ReTx Packet")
#                         sendPacket(bus, CAN_BOOTLOADER_SERVER_ID, retxPacket) 
#                         continue

#                     # Retransmission packet
#                     if (packet.isRetxPacket()):
#                         log.debug("Received ReTx Packet, Retransmitting last packet")
#                         sendPacket(bus, CAN_BOOTLOADER_SERVER_ID, lastTxPacket)
#                         continue

#                     # Ack packet
#                     if (packet.isRetxPacket()):
#                         log.debug("Received ACK Packet")
#                         continue

#                     log.debug("Received normal packet")
#                     # Append packet to buffer
#                     packetBuffer.append(packet)

#                     # Send ACK
#                     log.debug("Sending ACK packet")
#                     sendPacket(bus, CAN_BOOTLOADER_SERVER_ID, ackPacket)

#             else:
#                 log.debug("Received invalid ID")

#         except asyncio.TimeoutError:
#             pass # prevent await blocking

def packetConstructor(bus, reader):
    global rxCanBuffer
    while (True):
        msg = reader.get_message(timeout=0.5)

        if (msg == None):
            break

        if (msg.arbitration_id == CAN_BOOTLOADER_PMB_ID):
            log.debug("Received CAN from PMB")
            rxCanBuffer = rxCanBuffer + msg.data
            
            # Try to build a Packet
            if (len(rxCanBuffer) >= PACKET_TOTAL_SIZE):
                # Extract data from buffer
                rawBytes: bytearray = readFromBuffer(PACKET_TOTAL_SIZE) 

                lenType = rawBytes[0].to_bytes()
                data = rawBytes[PACKET_LENTYPE_SIZE:PACKET_DATA_SIZE+1]
                crc = rawBytes[PACKET_DATA_SIZE+1:]

                packet = Packet(lenType, data, crc)
                computedCrc = packet.computeCrc()

                # Need Retransmission
                if (computedCrc != packet.crc):
                    log.error(f"Wrong CRC! Received: {packet.getCrc()}, Computed: {int.from_bytes(computedCrc, "little")}")
                    log.debug("Sending ReTx Packet")
                    sendPacket(bus, CAN_BOOTLOADER_SERVER_ID, retxPacket) 
                    continue

                # Retransmission packet
                if (packet.isRetxPacket()):
                    log.debug("Received ReTx Packet, Retransmitting last packet")
                    sendPacket(bus, CAN_BOOTLOADER_SERVER_ID, lastTxPacket)
                    continue

                # Ack packet
                if (packet.isRetxPacket()):
                    log.debug("Received ACK Packet")
                    break

                log.debug("Received normal packet")
                # Append packet to buffer
                packetBuffer.append(packet)

                # Send ACK
                log.debug("Sending ACK packet")
                sendPacket(bus, CAN_BOOTLOADER_SERVER_ID, ackPacket)
                break

        else:
            log.debug("Received invalid ID")

def stateMachine():
    while (len(packetBuffer)):
        packet = packetBuffer[0]
        del packetBuffer[0]

        print(packet)

async def main():
    with can.Bus(interface='canine', bitrate=1000000) as bus:
        # reader = can.AsyncBufferedReader()
        reader = can.BufferedReader()
        # logger = can.Logger("canMsg.log")
        listeners: list[MessageRecipient] = [reader]
        # loop = asyncio.get_running_loop()
        # notifier = can.Notifier(bus, listeners, loop=loop)
        notifier = can.Notifier(bus, listeners)
            
        try:
            # packetConstructor_task = asyncio.create_task(packetConstructor(bus, reader))
            global shutdown
            while (not shutdown):
                packetConstructor(bus, reader) # return when a packet is constructed or ack received
                stateMachine()
            
            # await packetConstructor_task
        except asyncio.CancelledError:
            log.debug("Shutting down notifier")
        finally:
            notifier.stop()

 
def handler(signal_received, frame):
    global shutdown
    log.critical("CTRL-c detected")
    for task in asyncio.all_tasks():
        task.cancel()

    shutdown = True


if __name__ == "__main__":
    # Initilise Logger
    log = get_logger("fw_log")
    log.info("Start FW Updator")
    # signal(SIGINT, handler)

    asyncio.run(main())
    while False:
        rx = input("Send CAN message \n"
                   "\t1: Enter APP \n"
                   "\t2: Echo\n"
                   "\t3: Invalid\n"
                   "\t4: Custom ID\n"
                   ">>")
        if (int(rx) == 1): # Jump
            send(1)  
        elif (int(rx) == 2): # Echo
            send(2)
        elif (int(rx) == 3): # Spam invalid data
            send(3)
        elif (int(rx) == 4): # Custom ID
            id = input("ID? ")
            send_data(int(id), [0xFF])
