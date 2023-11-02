#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

#include "firmware.h"
#include "system.h"
#include "uart.h"
#include "logger.h"

static void main_gpioInit(void);
static void main_canInit(void);

// static void main_display_setup(void);
// static void main_uart_debug_setup(void);
// static void main_bq_setup(void);

int main(void) {
    system_init();
    main_gpioInit();
    uart_init();
    main_canInit();

    while (1) {
        gpio_toggle(MAIN_NERROR_PORT, MAIN_NERROR_PIN);
        system_delay_ms(1000);
    }
    return 0;
}

static void main_canInit(void) {
    logger_printInfo("CAN Init");

    /* CAN1 Init
     * OFF  : ttcm, awum, nart, rflm, txfp, Silent, Loopback
     * ON   : abom
     * Rate : 1Mbps
     * - SJW : 1TQ
     * - TS1 : 13TQ
     * - TS2 : 2TQ
     * - BRP : 3 (of AHB, 48MHz)
     */
    can_init(BX_CAN1_BASE, false, true, false, false, false, false, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 3, false, false);
    
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
    logger_printSuccess("CAN Init Sucessful");
};


static void main_gpioInit(void) {
    /* Enable clock on GPIOB, GPIOC */
    logger_printInfo("GPIO Init");
    rcc_periph_clock_enable(RCC_GPIOA); 
    rcc_periph_clock_enable(RCC_GPIOB); 
    rcc_periph_clock_enable(RCC_GPIOC); 

    /**
     * PORT A Setup
     * USART1:
     *      PA9     UART_TX
     *      PA10    UART_RX
     * 
     */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, MAIN_UART1_TX_PIN | MAIN_UART1_RX_PIN);
    gpio_set_af(GPIOA, GPIO_AF1, MAIN_UART1_TX_PIN | MAIN_UART1_RX_PIN);

    /**
     * PORT B Setup
     * DIGITAL_INPUT:
     *      PB13    BQ_ALERT1
     *      PB12    BQ_ALERT2
     * DIGITAL_OUTPUT:
     *      PB14    NERROR          ON
     */
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, MAIN_BQ_ALERT1_PIN | MAIN_BQ_ALERT2_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MAIN_NERROR_PIN);
    gpio_set(GPIOB, MAIN_NERROR_PIN); 

    /**
     * PORT C Setup
     * DIGITAL_INPUT: 
     *      PC7     REED_OFF
     * DIGITAL_OUTPUT:
     *      PC1     RELAY_ON            OFF
     *      PC3     PMOS_ON             OFF
     *      PC4     DISPLAY_RESET       OFF
     */
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, MAIN_REED_OFF_PIN);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MAIN_RELAY_ON_PIN | MAIN_PMOS_ON_GPIO_PIN | MAIN_DISPLAY_RESET_PIN);
    gpio_clear(GPIOC, MAIN_RELAY_ON_PIN | MAIN_PMOS_ON_GPIO_PIN| MAIN_DISPLAY_RESET_PIN); 

    logger_printSuccess("GPIO init successful");
}