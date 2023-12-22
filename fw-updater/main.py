#!/usr/bin/env python

from time import sleep
import can
from canine import CANineBus

BOOTLOADER_SERVER_ID = 40
CMD_JUMP = 0xAA
CMD_ECHO = 0xBB

SLEEP_TIME = 0.1
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
        
        msg = can.Message(
            arbitration_id=BOOTLOADER_SERVER_ID, data=data_to_send, is_extended_id=False
        )

        try:
            bus.send(msg)
            print(f"Message sent on {bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")

def send_data(id, data):
    with can.Bus(interface='canine', bitrate=1000000) as bus:
        msg = can.Message(
            arbitration_id=id, data=data, is_extended_id=False
        )

        try:
            bus.send(msg)
            print(f"Message sent on {bus.channel_info}")
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
    while True:
        rx = input("Send CAN message \n"
                   "\t1: Enter APP \n"
                   "\t2: Echo\n"
                   "\t3: Invalid\n"
                   "\t4: 80 Bytes\n"
                   "\t5: Custom ID\n"
                   ">>")
        if (int(rx) == 1): # Jump
            send(1)  
        elif (int(rx) == 2): # Echo
            send(2)
        elif (int(rx) == 3): # Spam invalid data
            send(3)
        elif (int(rx) == 4): # Spam msg
            send_data(BOOTLOADER_SERVER_ID, [0x11]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0x22]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0x33]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0x44]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0x55]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0x66]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0x77]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0x88]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0x99]*8)
            sleep(SLEEP_TIME)
            send_data(BOOTLOADER_SERVER_ID, [0xAA]*8)
        elif (int(rx) == 5): # Custom ID
            id = input("ID? ")
            send_data(int(id), [0xFF])
