#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

#include "firmware.h"
#include "system.h"
#include "uart.h"
#include "can.h"
#include "log.h"
#include "gpio.h"
#include "i2c.h"
#include "ADS1115.h"
#include "retarget.h"

float PMB_getPressure(void);
static void setup(void);

int main(void) {
    setup();

    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};

    while (true) {
        gpio_toggle(PMB_NERROR_PORT, PMB_NERROR_PIN);
        
        volatile float pressure = PMB_getPressure();
        log_pInfo("Pressure: %f", pressure);

        uint32_t data_size = sizeof(data);
        log_pInfo("Sending CAN message of size: %d", data_size);
        volatile int8_t res = can_transmit(BX_CAN1_BASE, 0x1, false, false, data_size, data);
        if (res >= 0) {
            log_pInfo("CAN: Data sent");
        } else {
            log_pInfo("CAN: Failed to send");
        }
        
        PMB_system_delayMs(3000);
    }
    return 0;
}

static void setup(void) {
    PMB_system_init();    
    PMB_uart_init();
    retarget_init();
    PMB_gpio_init();

    while(!PMB_can_init()) PMB_system_delayMs(1000);
    while(!PMB_i2c_init()) PMB_system_delayMs(1000);
}

float PMB_getPressure(void) {
	return (ADS_ReadADC_SingleEnded(1)*0.0001875) / (5 * 0.0040) + 10;
}