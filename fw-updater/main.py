#!/usr/bin/env python
import can
from canine import CANineBus

BOOTLOADER_SERVER_ID = 40
CMD_JUMP = 0xAA
CMD_ECHO = 0xBB

def send_one(opt):
    """Sends a single message."""

    # this uses the default configuration (for example from the config file)
    # see https://python-can.readthedocs.io/en/stable/configuration.html
    with can.Bus(interface='canine', bitrate=1000000) as bus:

        if opt == 2: # Echo
            msg = can.Message(
                arbitration_id=BOOTLOADER_SERVER_ID, data=[CMD_ECHO], is_extended_id=False
            )
        elif opt == 1: # Jump to Main
            msg = can.Message(
                arbitration_id=BOOTLOADER_SERVER_ID, data=[CMD_JUMP], is_extended_id=False
            )
        # data_bank = [0xAA]
        # for i in data_bank:
        #     msg = can.Message(
        #         arbitration_id=40, data=[i], is_extended_id=False
        #     )

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
        rx = input("Send CAN message \n\t1: Enter APP \n\t2: Echo\n>>")
        if (int(rx) == 1):
            send_one(1)  
        elif (int(rx) == 2):
            send_one(2)