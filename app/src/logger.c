#include "logger.h"
#include "uart.h"

#include <stdio.h>
// #define UART_WRITE_STRING_FUNC(x)      (PMB_uart1_writeString(x))

void PMB_logger_printInfo(char* info) {
printf("[INFO]: ");
printf(info);
printf("\r\n");
}

void PMB_logger_printSuccess(char* info) {
printf("[OK]: ");
printf(info);
printf("\r\n");
}

void PMB_logger_printError(char* info) {
printf("[ERROR]: ");
printf(info);
printf("\r\n");
}