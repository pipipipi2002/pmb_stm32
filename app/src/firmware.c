#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

#include "firmware.h"
#include "system.h"
#include "uart.h"
#include "logger.h"

static void PMB_commonGpioInit(void);
static uint8_t PMB_canInit(void);

// static void main_display_setup(void);
// static void main_uart_debug_setup(void);
// static void main_bq_setup(void);

int main(void) {
    PMB_system_init();    
    PMB_uart_init();
    PMB_commonGpioInit();
    while(PMB_canInit()) {
        PMB_system_delayMs(1000);
    }
    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};

    while (1) {
        gpio_toggle(PMB_NERROR_PORT, PMB_NERROR_PIN);
        volatile uint32_t data_size = sizeof(data);
        PMB_logger_printInfo("Sending CAN");
        volatile int8_t res = can_transmit(BX_CAN1_BASE, 0x1, false, false, data_size, data);
        if (res >= 0) {
            PMB_logger_printSuccess("CAN: Data sent");
        } else {
            PMB_logger_printError("CAN: Failed to send");
        }
        PMB_system_delayMs(5000);
    }
    return 0;
}

static uint8_t PMB_canInit(void) {
    PMB_logger_printInfo("CAN Init");
    
    rcc_periph_clock_enable(RCC_CAN);
    can_reset(BX_CAN1_BASE);

    /* CAN1 Init
     * OFF  : ttcm, awum, nart, rflm, txfp, Silent, Loopback
     * ON   : abom
     * Rate : 1Mbps
     * - SJW : 1TQ
     * - TS1 : 13TQ
     * - TS2 : 2TQ
     * - BRP : 3 (of AHB, 48MHz)
     */
    uint32_t ret = can_init(BX_CAN1_BASE, false, true, false, false, false, false, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 3, false, false);
    if (ret == 1) {
        PMB_logger_printError("CAN init failed");
        return 1;
    }
    
	/* ID 0 ~ 15 goes to FIFO0
	 * Filter configuration:
	 * ID: 		0b00001111
	 * Mask:	0b11110000
	 * Effect:  Accept all IDs below 16
	 */
    can_filter_id_mask_16bit_init(1, 0x0F, 0xF0, 0x0F, 0xF0, CAN_FIFO0, true);

    /* ID 16 ~  goes to FIFO1
	 * Filter configuration:
	 * ID: 		0x00
	 * Mask:	0x00
	 * Effect:  Accept all. But since 0~15 will go into filter bank with higher priority,
	 * only 16~ will go here
	 */
    can_filter_id_mask_16bit_init(2, 0x00, 0x00, 0x00, 0x00, CAN_FIFO1, true);
    PMB_logger_printSuccess("CAN Init Sucessful");
    return 0;
};


static void PMB_commonGpioInit(void) {
    PMB_logger_printInfo("Common GPIO Init");
    /**
     * PORT A Setup
     * USART1:
     *      PA9     UART_TX
     *      PA10    UART_RX
     * CAN1:
     *      PA11    CAN_RX
     *      PA12    CAN_TX
     */

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, PMB_CAN_RX_PIN | PMB_CAN_TX_PIN);
    gpio_set_af(GPIOA, GPIO_AF4, PMB_CAN_TX_PIN | PMB_CAN_RX_PIN);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PMB_CAN_RX_PIN | PMB_CAN_TX_PIN);

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

    PMB_logger_printSuccess("GPIO init successful");
}