# Features
This firmware includes the functionality provided by the code ported over from BBAS. Telemetry includes
- Voltage 
- Current
- Pressure
- Temperature
- Heartbeat

Moreover, it is able to respond to Firmware Information request through CAN with the following frame
- ID: 42
- DATA0: 26; DATA1: 218; DATA2: 333; DATA3: 192

It will respond with a CAN frame:
- ID: 42
- DATA0-3: uint32_t DEVICE_ID (little endian)
- DATA4-7: uint32_t COMMIT_VERSION (little endian)