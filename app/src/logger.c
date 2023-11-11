#include "logger.h"
#include "uart.h"

#include <stdio.h>
#include <stdarg.h>
// #define UART_WRITE_STRING_FUNC(x)      (PMB_uart1_writeString(x))

void logger_printInfo(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    printf("[INFO]: ");
    vprintf(fmt, args);
    printf("\n\r");
}

void logger_printSuccess(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    printf("[OK]: ");
    vprintf(fmt, args);
    printf("\n\r");
}

void logger_printError(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    printf("[ERROR]: ");
    vprintf(fmt, args);
    printf("\n\r");
}