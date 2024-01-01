#ifndef INC_PMB_FIRMWARE_H
#define INC_PMB_FIRMWARE_H

#include "common_defines.h"
#include "log.h"

#define FIRMWARE_DISABLE_DEBUG

#if defined (FIRMWARE_DISABLE_DEBUG)
	#define $INFO(fmt, ...)
	#define $SUCCESS(fmt, ...) log_pSuccess(fmt, ##__VA_ARGS__)
	#define $ERROR(fmt, ...) log_pError(fmt, ##__VA_ARGS__)
#elif defined (USE_LOGGER)
	#define $INFO(fmt, ...) log_pInfo(fmt, ##__VA_ARGS__)
	#define $ERROR(fmt, ...) log_pError(fmt, ##__VA_ARGS__)
	#define $SUCCESS(fmt, ...) log_pSuccess(fmt, ##__VA_ARGS__)
#else
	#include <stdio.h>
	#define $INFO(fmt, ...) printf(fmt, ##__VA_ARGS__)
	#define $ERROR(fmt, ...) printf(fmt, ##__VA_ARGS__)
	#define $SUCCESS(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif // USE_LOGGER

#endif // INC_PMB_FIRMWARE_H
