#!/usr/bin/env python

from time import sleep
import can
from canine import CANineBus
import crcmod
from multipledispatch import dispatch

CAN_BOOTLOADER_SERVER_ID = 40
CAN_BOOTLOADER_PMB_ID = 41

CMD_JUMP = 0xAA
CMD_ECHO = 0xBB

PACKET_LENTYPE_SIZE = 1
PACKET_DATA_SIZE = 16
PACKET_CRC_SIZE = 4
PACKET_TOTAL_SIZE = PACKET_LENTYPE_SIZE + PACKET_DATA_SIZE + PACKET_CRC_SIZE 

PACKET_TYPE_NORMAL = 1
PACKET_TYPE_UTILITY = 2

PACKET_UTILITY_ACK_DATA = 0xAA
PACKET_UTILITY_RETX_DATA = 0x55

SLEEP_TIME = 0.1 # interval between can frames

def computeCrc32(data:bytearray) -> bytes:
    crc32 = crcmod.mkCrcFun(poly=0x104C11DB7, rev=True, initCrc=0, xorOut=0xFFFFFFFF)
    result = crc32(data)
    return result.to_bytes(4, "little")

class Packet:
    def __init__(self, lenType:bytes=None, data:bytearray=None, crc:bytearray=None):
        self.lenType = lenType
        self.data = data
        
        if (crc == None and not (lenType == None or data == None)):
            self.crc = self.computeCrc()
        else:
            self.crc = crc

    def getLength(self) -> int:
        return (int.from_bytes(self.lenType, "little") >> 2)
    
    def getType(self) -> int:
        return (int.from_bytes(self.lenType, "little") & 0b11)
    
    def getLenType(self) -> bytes:
        return self.lenType
    
    @dispatch(length=int, type=int)
    def setLenType(self, length, type):
        self.lenType = Packet.constructLenType(length, type)

    @dispatch(lenType=bytes)
    def setLentype(self, lenType):
        self.lenType = lenType

    def getData(self) -> bytearray:
        return self.data
    
    @dispatch(data=bytearray)
    def setData(self, data:bytearray):
        self.data = data

    @dispatch(data=bytes)
    def setData(self, data:bytes):
        self.data = bytearray(data)

    def getCrc(self) -> bytearray:
        return self.crc
    
    def setCrc(self):
        self.crc = self.computeCrc()
        
    def computeCrc(self) -> bytearray:
        data = self.lenType + self.data
        return bytearray(computeCrc32(data))
    
    @staticmethod
    def constructLenType(length:int, type:int) -> bytes:
        lenType = (length << 2) | type
        return lenType.to_bytes(1, "little")

        
ackPacket = Packet(Packet.constructLenType(PACKET_LENTYPE_SIZE, PACKET_TYPE_UTILITY), PACKET_UTILITY_ACK_DATA.to_bytes())
retxPacket = Packet(Packet.constructLenType(PACKET_LENTYPE_SIZE, PACKET_TYPE_UTILITY), PACKET_UTILITY_RETX_DATA.to_bytes())
lastTxPacket = Packet()



def send(opt):
    """Sends a single message."""

    # this uses the default configuration (for example from the config file)
    # see https://python-can.readthedocs.io/en/stable/configuration.html
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


if __name__ == "__main__":

    lenType = [0b00000110]
    data = [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x11, 0x22] # CRC is 2502068239 
    new_packet = Packet(bytes(lenType), bytearray(data))

    print(new_packet.getLength())
    print(new_packet.getType())
    print(int.from_bytes(new_packet.getCrc(), "little"))

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
