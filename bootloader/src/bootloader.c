// Libopencm3 Header Files
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/memorymap.h>

// Shared Header Files
#include "common_defines.h"
#include "board_def.h"
#include "system.h"
#include "uart.h"
#include "log.h"
#include "gpio.h"
#include "can.h"

/*
 * Global variables
 */
uint64_t activityTimer = 0;

/*
 * Internal Function Declarations
 */
int main (void);
static void setup(void);
static void destruct(void);
static void jumpToApplication(void);

/*
 * Function Definitions
 */
int main (void) {
    system_rccInit();
    system_systickInit();
    setup();
    log_pInfo("Entered Bootloader");
    log_pInfo("Power Monitoring Board AUV4");

    /* Supply power */
    gpio_clear(PMB_RELAY_OFF_PORT, PMB_RELAY_OFF_PIN);          // Latch pwoer to PMB 
    gpio_clear(PMB_PMOS_ON_GPIO_PORT, PMB_PMOS_ON_GPIO_PIN);    // Disable power to vehicle
    log_pInfo("Latch power to circuit, disable power to AUV4");

    log_pInfo("Prepare to enter application");
    destruct();
    jumpToApplication();
    return 0; // will not reach here
}

static void setup(void) {
    while(!uart1_setup()) system_delayMs(1000);
    while(!gpio_setup()) system_delayMs(1000);
    while(!can_setup()) system_delayMs(1000);

    log_pSuccess("Setup Completed");
}
static void destruct(void) {
    uart1_destruct();
    system_systickDeinit();
}

static void jumpToApplication(void) {
    typedef void (*void_fn)(void); // Function Pointer

    /* vector table is after SP; Reset handler is 1st in the Vector Table */
    uint32_t* reset_vector_addr = (uint32_t*) (MAIN_APP_START_ADDR + 4U);
    /* The reset handler contains the address to the reset function */ 
    uint32_t* reset_handler_addr = (uint32_t*) (*reset_vector_addr); 
    /* Map reset vector to a function */
    void_fn app_reset_handler = (void_fn) reset_handler_addr;
    /* Set MSP to the APP stack pointer */
    uint32_t app_top_stack = (*(uint32_t*)MAIN_APP_START_ADDR); 
    __asm volatile ("MSR msp, %0" : : "r" (app_top_stack) : );
    /* Jump to Main Application Reset Handler */
    app_reset_handler();
}
