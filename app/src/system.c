#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/rcc.h>

#include "system.h"

static void PMB_system_rccInit(void);
static void PMB_system_systickInit(void);

static volatile uint64_t system_ticks = 0;

/**
 * @brief ISR handler for systick.
 * @details Triggers every 1ms.
 */
void sys_tick_handler(void) {
    system_ticks++;
}

static void PMB_system_rccInit(void) {
    // Enable clock gating to GPIO PORT
    rcc_periph_clock_enable(RCC_GPIOA); 
    rcc_periph_clock_enable(RCC_GPIOB); 
    rcc_periph_clock_enable(RCC_GPIOC); 
    
    /* Set System frequency to 48 MHz, with HSI clock */
    rcc_clock_setup_in_hsi48_out_48mhz();

    /* I2C Clock source */
    rcc_osc_on(RCC_HSI);
    rcc_wait_for_osc_ready(RCC_HSI);
}

static void PMB_system_systickInit(void) {
    systick_set_frequency(SYSTICK_FREQ, rcc_ahb_frequency);
    systick_counter_enable();
    systick_interrupt_enable();
}

uint64_t PMB_system_getTicks(void) {
    return system_ticks;
}

void PMB_system_delayMs(uint64_t time_ms) {
    uint64_t end_time = PMB_system_getTicks() + time_ms;
    while (PMB_system_getTicks() < end_time);
}

void PMB_system_init(void){
    PMB_system_rccInit();
    PMB_system_systickInit();
}
