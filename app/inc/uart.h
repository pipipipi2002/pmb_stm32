#ifndef INC_PMB_UART_H
#define INC_PMB_UART_H

#include "common_defines.h"

void PMB_uart1_writeByte(uint8_t data);
void PMB_uart1_writeBytes(uint8_t* data, const uint32_t length);
void PMB_uart1_writeString(char* ptr);
void PMB_uart_init(void);
void PMB_uart_deinit(void);


#endif // INC_PMB_UART_H