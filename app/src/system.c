#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/rcc.h>

#include "system.h"
#include "common_defines.h"

static void system_rccInit(void);
static void system_systickInit(void);

static volatile uint64_t system_ticks = 0;

/**
 * @brief ISR handler for systick.
 * @details Triggers every 1ms.
 */
void sys_tick_handler(void) {
    system_ticks++;
}

static void system_rccInit(void) {
    /* Set System frequency to 48 MHz, with HSI clock */
    rcc_clock_setup_in_hsi48_out_48mhz();
}

static void system_systickInit(void) {
    systick_set_frequency(SYSTICK_FREQ, rcc_ahb_frequency);
    systick_counter_enable();
    systick_interrupt_enable();
}

uint64_t system_getTicks(void) {
    return system_ticks;
}

void system_delay_ms(uint64_t time_ms) {
    uint64_t end_time = system_getTicks() + time_ms;
    while (system_getTicks() < end_time);
}

void system_init(void){
    system_rccInit();
    system_systickInit();
}
