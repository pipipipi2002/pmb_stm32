#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/rcc.h>

#include "system.h"
#include "common_defines.h"

static void system_rcc_setup(void);
static void system_systick_setup(void);

static volatile uint64_t system_ticks = 0;

/**
 * @brief ISR handler for systick.
 * @details Triggers every 1ms.
 */
void sys_tick_handler(void) {
    system_ticks++;
}

static void system_rcc_setup(void) {
    /* Set System frequency to 48 MHz, with HSI clock */
    rcc_clock_setup_in_hsi48_out_48mhz();
}

static void system_systick_setup(void) {
    systick_set_frequency(SYSTICK_FREQ, rcc_ahb_frequency);
    systick_counter_enable();
    systick_interrupt_enable();
}

uint64_t system_get_ticks(void) {
    return system_ticks;
}

void system_delay_ms(uint64_t time_ms) {
    uint64_t end_time = system_get_ticks() + time_ms;
    while (system_get_ticks() < end_time);
}

void system_init(void){
    system_rcc_setup();
    system_systick_setup();
}
