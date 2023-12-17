#!/usr/bin/env python
import can
from canine import CANineBus

def send_one(opt):
    """Sends a single message."""

    # this uses the default configuration (for example from the config file)
    # see https://python-can.readthedocs.io/en/stable/configuration.html
    with can.Bus(interface='canine', bitrate=1000000) as bus:

        # if opt == 1:
        #     msg = can.Message(
        #         arbitration_id=40, data=[0xBB], is_extended_id=False
        #     )
        # elif opt == 2:
        #     msg = can.Message(
        #         arbitration_id=40, data=[0xAA], is_extended_id=False
        #     )
        data_bank = [0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA]
        for i in data_bank:
            msg = can.Message(
                arbitration_id=40, data=[i], is_extended_id=False
            )

            try:
                print(hex(i))
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
        rx = input("send?")
        if (int(rx) == 1):
            send_one(1)  
        elif (int(rx) == 2):
            send_one(2)