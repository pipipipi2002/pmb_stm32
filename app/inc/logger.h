#ifndef INC_LOGGER_H
#define INC_LOGGER_H

void logger_printInfo(const char* fmt, ...);
void logger_printSuccess(const char* fmt, ...);
void logger_printError(const char* fmt, ...);

#endif // INC_LOGGER_H