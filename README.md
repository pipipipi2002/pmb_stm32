# Power Monitoring Board STM32F072RB
This project is an adaptation of the Bumblebee Autonomous System's Power Monitoring Board system, a system that I primarily worked on for the Robosub 2023 competition.

This personal project aims to switch from STM32 CMSIS HAL to LibOpenCM3 library, as well as to implement other advanced feature such as CAN boot flashloader, error logging and storage, better graphics, RTOS and etc

Another reason for this project is to learn Rust. The master branch shall be kept in C, while another branch will be in Rust (to be added).

## Feature list / Wish list
- Port over current status to LibOpenCM3 (DONE)
- Implement Error logging (DONE) and storage to flash
- Implement CAN Boot Flashloader (maybe boot flash updater)
- Implement graphics using LVGL library
- RTOS
- RUST

# Setup
```bash
git clone https://github.com/pipipipi2002/pmb_stm32
git submodule init
git submodule update

cd libopencm3
make

cd ../app
make
```