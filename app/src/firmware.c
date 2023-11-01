#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "firmware.h"
#include "system.h"
#include "uart.h"


static void main_gpio_init(void);

// static void main_display_setup(void);
// static void main_uart_debug_setup(void);
// static void main_bq_setup(void);


static void main_gpio_init(void) {
    /* Enable clock on GPIOB, GPIOC */
    rcc_periph_clock_enable(RCC_GPIOA); 
    rcc_periph_clock_enable(RCC_GPIOB); 
    rcc_periph_clock_enable(RCC_GPIOC); 

    /**
     * PORT A Setup
     * USART1:
     *      PA9     UART_TX
     *      PA10    UART_RX
     * 
     */

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, MAIN_UART1_TX_PIN | MAIN_UART1_RX_PIN);
    gpio_set_af(GPIOA, GPIO_AF1, MAIN_UART1_TX_PIN | MAIN_UART1_RX_PIN);

    /**
     * PORT B Setup
     * DIGITAL_INPUT:
     *      PB13    BQ_ALERT1
     *      PB12    BQ_ALERT2
     * DIGITAL_OUTPUT:
     *      PB14    NERROR          ON
     */
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, MAIN_BQ_ALERT1_PIN | MAIN_BQ_ALERT2_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MAIN_NERROR_PIN);
    gpio_set(GPIOB, MAIN_NERROR_PIN); 

    /**
     * PORT C Setup
     * DIGITAL_INPUT: 
     *      PC7     REED_OFF
     * DIGITAL_OUTPUT:
     *      PC1     RELAY_ON            OFF
     *      PC3     PMOS_ON             OFF
     *      PC4     DISPLAY_RESET       OFF
     */
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, MAIN_REED_OFF_PIN);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MAIN_RELAY_ON_PIN | MAIN_PMOS_ON_GPIO_PIN | MAIN_DISPLAY_RESET_PIN);
    gpio_clear(GPIOC, MAIN_RELAY_ON_PIN | MAIN_PMOS_ON_GPIO_PIN| MAIN_DISPLAY_RESET_PIN); 
}

int main(void) {
    system_init();
    main_gpio_init();
    uart_init();

    while (1) {
        gpio_toggle(MAIN_NERROR_PORT, MAIN_NERROR_PIN);
        uart1_write_string("Toggle on\r\n");
        system_delay_ms(1000);
    }
    return 0;
}