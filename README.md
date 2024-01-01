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
git clone https://github.com/pipipipi2002/pmb_stm32
git submodule init
git submodule update

cd libopencm3
make

./build_all.sh
```