#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

#include "firmware.h"
#include "system.h"
#include "uart.h"
#include "can.h"
#include "logger.h"
#include "gpio.h"

int main(void) {
    PMB_system_init();    
    PMB_uart_init();
    PMB_gpio_init();
    while(PMB_can_init()) {
        PMB_system_delayMs(1000);
    }

    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};

    while (true) {
        gpio_toggle(PMB_NERROR_PORT, PMB_NERROR_PIN);
        
        PMB_logger_printInfo("Sending CAN");
        volatile uint32_t data_size = sizeof(data);
        volatile int8_t res = can_transmit(BX_CAN1_BASE, 0x1, false, false, data_size, data);
        if (res >= 0) {
            PMB_logger_printSuccess("CAN: Data sent");
        } else {
            PMB_logger_printError("CAN: Failed to send");
        }
        
        PMB_system_delayMs(5000);
    }
    return 0;
}