#ifndef INC_PMB_UART_IF_H
#define INC_PMB_UART_IF_H

#include "common_defines.h"

void uart1if_writeByte(uint8_t data);
uint32_t uart1if_writeBytes(uint8_t* data, const uint32_t length);
uint8_t uart1if_readByte(void);
uint32_t uart1if_readBytes(uint8_t* data, const uint32_t len);
bool uart1if_setup(void);
bool uart1if_destruct(void);

#endif // INC_PMB_UART_IF_H