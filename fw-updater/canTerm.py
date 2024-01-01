import can
import asyncio
from time import sleep
from canine import CANineBus
from typing import List

SLEEP_TIME = 0.001

def sendData(bus:can.Bus, id:int, data:bytearray, len:int) -> int:
    frames_sent:int = 0
    i:int = 0
    while (i < len):
        len_to_send = 8 if (i+8 < len) else (len - i)

        msg = can.Message(arbitration_id=id, data=data[i:i+len_to_send], is_extended_id=False)
        
        try:
            bus.send(msg)
            print(f"Send message: {msg}")
            sleep(SLEEP_TIME)
        except can.CanError:
            print("Message Failed to send")
        i += 8
        frames_sent += 1

    return frames_sent

async def receiveCanMessage(reader):
    while True:
        msg:can.Message = await reader.get_message()
        print(f"Received: {msg}")

async def sendCanMessage(bus):
    while True:
        input_id = await asyncio.to_thread(input, "Enter Dest CAN ID: ")
        input_id = int(input_id)
        input_len = await asyncio.to_thread(input, "Enter Data Length: ")
        input_len = int(input_len)
        if (input_len <= 0):
            print("Length should be positive")
            continue
        input_data: List[bytearray] = []
        for i in range (input_len):
            data_byte = await asyncio.to_thread(input, f"Enter byte {i}: ")
            data_byte = int(data_byte)
            input_data += int.to_bytes(data_byte, 1)

        sendData(bus, input_id, input_data, input_len)

async def startCanBus():
    with can.Bus(interface='canine', bitrate=1000000) as bus:
        reader = can.AsyncBufferedReader()
        notifier = can.Notifier(bus, [reader], loop=asyncio.get_running_loop())
        filters = [{"can_id": 42, "can_mask": 0xFF, "extended": False}]
        bus.set_filters(filters)
        
        await asyncio.gather(
            receiveCanMessage(reader),
            sendCanMessage(bus)
        )

        
if __name__ == "__main__":
    asyncio.run(startCanBus())