Refer to [CANine Documentation](https://canine.readthedocs.io/en/latest/canine.html#introduction) for installation of the hardware drivers to communicate with CANine.

# Setting up python environment
Using Conda or other equivalent virtual environment
```bash
# In this working directory
conda create --name can-testing python=3.10 -y
conda activate can-testing
pip install -r requirements.txt
```

# Using the Script
## Firmware Updator - `main.py`
Make sure the firmware `firmware.bin` has been built beforehand and exists in the `~/app/` directory. Running it is as simple as:
```Python
python main.py
```
The script will wait for the heartbeat message from the board to start the uploading process.
Moreover, this script extracts and injects information into the firmware binary.
- Extracts the device ID from a fixed region (through memory address). The device ID extracted will be used to validate the correct bootloader it is communicating with.
- Injects the commit hash (truncated to 32-bit) of the current commit

## CAN Terminal - `CanTerm.py`
- Able to monitor and send can data. 
- Edit the `CanTerm.py` script to adjust the filter and mask (line 54).

Running it
```Python
python CanTerm.py
```
