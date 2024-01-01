# Power Monitoring Board STM32F072RB
This project is an adaptation of the Bumblebee Autonomous System's Power Monitoring Board system, a system that I primarily worked on for the Robosub 2023 competition.

This personal project aims to switch from STM32 CMSIS HAL to LibOpenCM3 library, as well as to implement other advanced feature such as CAN boot flashloader, error logging and storage, better graphics, RTOS and etc

## Feature list / Wish list
- Port over current system to LibOpenCM3 (DONE)
- Implement Logging module (DONE) 
- Implement CAN Boot Flashloader (DONE)
- Implement Storage Logging Module
- Implement graphics using LVGL library
- RTOS

# Setup
Pre-reqs
- git
- ARM GNU Toolchain
- Make
- Python
- Conda
- OpenOCD
- ST-Utils
- VSCODE: Cortex Debug extension

```bash
# Repository Setup
git clone https://github.com/pipipipi2002/pmb_stm32
git submodule init
git submodule update

# Build the LibOpenCm3 Library once
cd libopencm3
make

# Test full build
./build_all.sh
```

# Flashing Instructions
Since this is a Single Bank system, with Bootloader and Main application residing in the memory region, we need to first flash the bootloader through the SWD port using a debugger, and then use the bootloader to flash the main application.

For this release, you will be unable to flash the firmware together with the bootloader. Only the bootloader can be flashed through SWD, and the main application flashed through bootloader.

## Flashing Bootloader
1. Go to the bootloader directory and build the bootloader binary.
```bash
~/bootloader$ make clean && make 
```
2. Connect the debugger to the board and power the board.
3. On the Debug page of VS-Code, select "Debug Bootloader" and click play button.
4. The code shall be flashed to the board and the program should be in a "halted" state.
5. Disconnect the debugger from the board.

## Flashing Main Application 
1. Go to the app directory and build the firmware binary.
```bash
~/app$ make clean && make
```
2. Connect the Canine CAN to USB converter to the board and to your PC.
3. Go to the fw-updator directory, activate conda environment, and run the firmware updator python code. The python code will wait for a heartbeat from the board. 
```bash
~/fw-updator$ conda activate can-testing
~/fw-updator$ python main.py
```
4. Power your board and the firmware will be uploaded and flashed to the board.

# Debugging
Both Main Application and Bootloader can be debugged through the SWD Port (but not both at the same time). This can be done through the VS-Code Debug page, select "Attach to Bootloader" to debug bootloader, or "Attach to Application" to debug Main Application.

There is also the UART output, which will print out the logs as set in the firmwares. This UART output is set with baudrate of `115200` and `8N1` configuration. It is not able to receive any data. This UART output is useful for when the bootloader is unable to jump to the main application or for main application statuses.

# Resources
Refer to the ![drawio](https://app.diagrams.net/#G1f-3hg4P5fgFP96kJSaKQO0dy9FH2EILR) diagram for more information about the bootloading mechanics◊