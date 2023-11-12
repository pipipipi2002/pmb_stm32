#include <libopencm3/stm32/gpio.h>

#include "gpio.h"
#include "log.h"
#include "firmware.h"

void PMB_gpio_init(void) {
    log_pInfo("GPIO Init");

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

    log_pSuccess("GPIO Init successful");
}