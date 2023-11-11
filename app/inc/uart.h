#ifndef INC_PMB_UART_H
#define INC_PMB_UART_H

#include "common_defines.h"

void PMB_uart1_writeByte(uint8_t data);
uint32_t PMB_uart1_writeBytes(uint8_t* data, const uint32_t length);
uint8_t PMB_uart1_readByte(void);
uint32_t PMB_uart1_readBytes(uint8_t* data, const uint32_t len);
void PMB_uart_init(void);
void PMB_uart_deinit(void);

#endif // INC_PMB_UART_H