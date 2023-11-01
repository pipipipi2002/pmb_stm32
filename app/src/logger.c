#include "logger.h"
#include "uart.h"

#define UART_WRITE_STRING_FUNC(x)      (uart1_write_string(x))

void logger_printInfo(char* info) {
    UART_WRITE_STRING_FUNC("[INFO]: ");
    UART_WRITE_STRING_FUNC(info);
    UART_WRITE_STRING_FUNC("\r\n");
}

void logger_printSuccess(char* info) {
    UART_WRITE_STRING_FUNC("[OK]: ");
    UART_WRITE_STRING_FUNC(info);
    UART_WRITE_STRING_FUNC("\r\n");
}

void logger_printError(char* info) {
    UART_WRITE_STRING_FUNC("[ERR]: ");
    UART_WRITE_STRING_FUNC(info);
    UART_WRITE_STRING_FUNC("\r\n");
}