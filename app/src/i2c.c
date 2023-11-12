#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "i2c.h"
#include "firmware.h"
#include "log.h"

bool PMB_i2c_init(void) {
    log_pInfo("I2C Init");

    /* GPIO configuration */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, PMB_I2C1_SCL_PIN | PMB_I2C1_SDA_PIN);
    gpio_set_af(GPIOB, GPIO_AF1, PMB_I2C1_SCL_PIN | PMB_I2C1_SDA_PIN);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, PMB_I2C1_SCL_PIN | PMB_I2C1_SDA_PIN);
    
    /* Turn on HSI for I2C */
    rcc_set_i2c_clock_hsi(I2C1);
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_reset_pulse(RST_I2C1); // This function solves delayed ACK problem

    /* I2C Register Configuration */
    i2c_peripheral_disable(I2C1);
    
    i2c_set_speed(I2C1, i2c_speed_sm_100k, 8); // HSI8 is 8 MegaHertz
    i2c_set_7bit_addr_mode(I2C1);
    i2c_enable_analog_filter(I2C1);
    i2c_enable_stretching(I2C1);

    i2c_peripheral_enable(I2C1);
    log_pSuccess("I2C Init Completed");
    return true;
}
