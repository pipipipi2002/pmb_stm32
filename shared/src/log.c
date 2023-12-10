#include "log.h"

#include <stdio.h>
#include <stdarg.h>

void log_pInfo(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    printf("[INFO]: ");
    vprintf(fmt, args);
    printf("\n\r");
}

void log_pSuccess(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    printf("[OK]: ");
    vprintf(fmt, args);
    printf("\n\r");
}

void log_pError(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    printf("[ERROR]: ");
    vprintf(fmt, args);
    printf("\n\r");
}