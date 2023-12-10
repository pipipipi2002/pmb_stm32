// Libopencm3 Header Files
#include <libopencm3/stm32/gpio.h>

// Shared Header Files
#include "common_defines.h"
#include "board_def.h"
#include "system.h"
#include "uart.h"
#include "log.h"
#include "gpio.h"
#include "retarget.h"
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
static inline void loop(void);

/*
 * Function Definitions
 */
int main (void) {
    setup();
    log_pInfo("Entered Bootloader");
    log_pInfo("Power Monitoring Board AUV4");

    /* Supply power */
    gpio_clear(PMB_RELAY_OFF_PORT, PMB_RELAY_OFF_PIN);          // Latch pwoer to PMB 
    gpio_clear(PMB_PMOS_ON_GPIO_PORT, PMB_PMOS_ON_GPIO_PIN);    // Disable power to vehicle
    log_pInfo("Latch power to circuit, disable power to AUV4");

    while (true) { loop(); }
    return 0; // will not reach here
}

void setup(void) {
    while(!system_setup());    
    while(!uart1_setup()) system_delayMs(1000);
    while(!retarget_setup()) system_delayMs(1000);
    while(!gpio_setup()) system_delayMs(1000);
    while(!can_setup()) system_delayMs(1000);

    log_pSuccess("Setup Completed");
}

static inline void loop(void) {
    if (system_getTicks() - activityTimer > 1000) {
        activityTimer = system_getTicks();
        gpio_toggle(PMB_NERROR_PORT, PMB_NERROR_PIN);
        log_pInfo("Heartbeat");
    }
}