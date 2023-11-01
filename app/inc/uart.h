#ifndef INC_UART_H
#define INC_UART_H

#include "common_defines.h"

void uart1_write_byte(uint8_t data);
void uart1_write_bytes(uint8_t* data, const uint32_t length);
void uart1_write_string(char* ptr);
void uart_init(void);
void uart_deinit(void);


#endif // INC_UART_H