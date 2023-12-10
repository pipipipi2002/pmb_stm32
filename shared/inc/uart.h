#ifndef INC_PMB_UART_H
#define INC_PMB_UART_H

#include "common_defines.h"

void uart1_writeByte(uint8_t data);
uint32_t uart1_writeBytes(uint8_t* data, const uint32_t length);
uint8_t uart1_readByte(void);
uint32_t uart1_readBytes(uint8_t* data, const uint32_t len);
bool uart1_setup(void);
bool uart1_destruct(void);

#endif // INC_PMB_UART_H