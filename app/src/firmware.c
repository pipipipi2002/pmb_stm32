#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

#include "firmware.h"
#include "system.h"
#include "uart.h"
#include "can.h"
#include "logger.h"


static void PMB_commonGpioInit(void);
static uint8_t PMB_canInit(void);

int main(void) {
    PMB_system_init();    
    PMB_uart_init();
    PMB_commonGpioInit();
    while(PMB_can_init()) {
        PMB_system_delayMs(1000);
    }
    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};

    while (1) {
        gpio_toggle(PMB_NERROR_PORT, PMB_NERROR_PIN);
        volatile uint32_t data_size = sizeof(data);
        PMB_logger_printInfo("Sending CAN");
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

static void PMB_commonGpioInit(void) {
    PMB_logger_printInfo("Common GPIO Init");
    /**
     * PORT A Setup
     * USART1:
     *      PA9     UART_TX
     *      PA10    UART_RX
     * CAN1:
     *      PA11    CAN_RX
     *      PA12    CAN_TX
     */



    /**
     * PORT B Setup
     * DIGITAL_INPUT:
     *      PB13    BQ_ALERT1
     *      PB12    BQ_ALERT2
     * DIGITAL_OUTPUT:
     *      PB14    NERROR          ON
     */
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PMB_BQ_ALERT1_PIN | PMB_BQ_ALERT2_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PMB_NERROR_PIN);
    gpio_set(GPIOB, PMB_NERROR_PIN); 

    /**
     * PORT C Setup
     * DIGITAL_INPUT: 
     *      PC7     REED_OFF
     * DIGITAL_OUTPUT:
     *      PC1     RELAY_ON            OFF
     *      PC3     PMOS_ON             OFF
     *      PC4     DISPLAY_RESET       OFF
     */
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PMB_REED_OFF_PIN);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PMB_RELAY_ON_PIN | PMB_PMOS_ON_GPIO_PIN | PMB_DISPLAY_RESET_PIN);
    gpio_clear(GPIOC, PMB_RELAY_ON_PIN | PMB_PMOS_ON_GPIO_PIN| PMB_DISPLAY_RESET_PIN); 

    PMB_logger_printSuccess("GPIO init successful");
}