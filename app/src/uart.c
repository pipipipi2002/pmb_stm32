#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>


#include "uart.h"
#include "firmware.h"

static void uart1_init(void);
static void uart1_deinit(void);

static void uart1_init(void) {
    rcc_periph_clock_enable(RCC_USART1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_databits(USART1, 8);
    usart_set_baudrate(USART1, BAUDRATE);
    usart_set_parity(USART1, 0);
    usart_set_stopbits(USART1, 1);

    usart_enable(USART1);
}

static void uart1_deinit(void) {
    usart_disable(USART1);
    rcc_periph_clock_disable(RCC_USART1);
}

void uart1_writeByte(uint8_t data) {
    usart_send_blocking(USART1, (uint16_t) data);
}

void uart1_writeBytes(uint8_t* data, const uint32_t length) {
    for (uint32_t i = 0;  i < length; i++) {
        uart1_writeByte(data[i]);
    }
}

void uart1_writeString(char* ptr) {
    uart1_writeBytes((uint8_t *) ptr, strlen(ptr));
}

void uart_init(void) {
    uart1_init();
}

void uart_deinit(void) {
    uart1_deinit();
}