#ifndef INC_LOG_H
#define INC_LOG_H

#define USE_LOGGER

void log_pInfo(const char* fmt, ...);
void log_pSuccess(const char* fmt, ...);
void log_pError(const char* fmt, ...);

#endif // INC_LOG_H