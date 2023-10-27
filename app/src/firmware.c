#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define LED_PIN GPIO5
#define LED_PORT GPIOA

static void main_rcc_setup(void) {
    /* Set System frequency to 84 MHz, with HSI clock */
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
}

static void main_gpio_setup(void) {
    /* Enable clock on GPIOB */
    rcc_periph_clock_enable(RCC_GPIOA); 

    /* Set pin mode output for LD2 (blue led)*/
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

static void main_delay_cycles(uint32_t u32DelayCycles) {
    while (u32DelayCycles--) {
        __asm__("nop");
    }
}

int main(void) {
    main_rcc_setup();
    main_gpio_setup();

    while (1) {
        gpio_toggle(LED_PORT, LED_PIN);
        main_delay_cycles(72000000 / 4);
    }
    return 0;
}