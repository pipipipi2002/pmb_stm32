// Libopencm3 Header Files
#include <libopencm3/stm32/gpio.h>

// Shared Header Files
#include "common_defines.h"
#include "board_def.h"
#include "system.h"
#include "log.h"
#include "uart_if.h"
#include "gpio_if.h"
#include "can_if.h"

#define CAN_ID_BOOTLOADER_SERVER        (40)
#define BOOTLOADER_SERVER_JUMP_CMD      0xAA
#define BOOTLOADER_SERVER_ECHO_CMD      0xBB
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

    while (1) {
        canFrame_ts canframe;
        log_pInfo("Waiting for command");
        while (!canif_getRxDataReady()) {
            system_delayMs(500);
        }

        canif_getRxData(&canframe);
        if (canframe.id == CAN_ID_BOOTLOADER_SERVER) {
            switch (canframe.data[0]) {
                case BOOTLOADER_SERVER_JUMP_CMD:
                    log_pInfo("Prepare to enter application");
                    destruct();
                    jumpToApplication();
                    break;
                case BOOTLOADER_SERVER_ECHO_CMD:
                    log_pInfo("ECHO");
                    break;
                default:
                    log_pError("Fall through default");
            }
        }

    }
    return 0; // will not reach here
}

static void setup(void) {
    while(!uart1if_setup()) system_delayMs(1000);
    while(!gpioif_setup()) system_delayMs(1000);
    while(!canif_setup()) system_delayMs(1000);

    log_pSuccess("Setup Completed");
}
static void destruct(void) {
    uart1if_destruct();
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
