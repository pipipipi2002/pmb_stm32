#include "logger.h"
#include "uart.h"

#define UART_WRITE_STRING_FUNC(x)      (PMB_uart1_writeString(x))

void PMB_logger_printInfo(char* info) {
    UART_WRITE_STRING_FUNC("[INFO]: ");
    UART_WRITE_STRING_FUNC(info);
    UART_WRITE_STRING_FUNC("\r\n");
}

void PMB_logger_printSuccess(char* info) {
    UART_WRITE_STRING_FUNC("[OK]: ");
    UART_WRITE_STRING_FUNC(info);
    UART_WRITE_STRING_FUNC("\r\n");
}

void PMB_logger_printError(char* info) {
    UART_WRITE_STRING_FUNC("[ERROR]: ");
    UART_WRITE_STRING_FUNC(info);
    UART_WRITE_STRING_FUNC("\r\n");
}