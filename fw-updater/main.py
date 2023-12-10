#!/usr/bin/env python
import can
from canine import CANineBus

def send_one():
    """Sends a single message."""

    # this uses the default configuration (for example from the config file)
    # see https://python-can.readthedocs.io/en/stable/configuration.html
    with can.Bus(interface='canine', bitrate=1000000) as bus:

        msg = can.Message(
            arbitration_id=0x01, data=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88], is_extended_id=False
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
        rx = input("send?")
        if (int(rx) == 1):
            send_one()  
        elif (int(rx) == 2):
            start_rx()