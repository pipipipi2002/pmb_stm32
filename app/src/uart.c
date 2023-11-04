#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "uart.h"
#include "firmware.h"

static void PMB_uart1_init(void);
static void PMB_uart1_gpioInit(void);
static void PMB_uart1_deinit(void);

static void PMB_uart1_init(void) {
    rcc_periph_clock_enable(RCC_USART1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_databits(USART1, 8);
    usart_set_baudrate(USART1, PMB_BAUDRATE);
    usart_set_parity(USART1, 0);
    usart_set_stopbits(USART1, 1);

    usart_enable(USART1);
}

static void PMB_uart1_gpioInit(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, PMB_UART1_TX_PIN | PMB_UART1_RX_PIN);
    gpio_set_af(GPIOA, GPIO_AF1, PMB_UART1_TX_PIN | PMB_UART1_RX_PIN);
}

static void PMB_uart1_deinit(void) {
    usart_disable(USART1);
    rcc_periph_clock_disable(RCC_USART1);
}

void PMB_uart1_writeByte(uint8_t data) {
    usart_send_blocking(USART1, (uint16_t) data);
}

void PMB_uart1_writeBytes(uint8_t* data, const uint32_t length) {
    for (uint32_t i = 0;  i < length; i++) {
        PMB_uart1_writeByte(data[i]);
    }
}

void PMB_uart1_writeString(char* ptr) {
    PMB_uart1_writeBytes((uint8_t *) ptr, strlen(ptr));
}

void PMB_uart_init(void) {
    PMB_uart1_gpioInit();
    PMB_uart1_init();
}

void PMB_uart_deinit(void) {
    PMB_uart1_deinit();
}