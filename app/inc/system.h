#ifndef INC_PMB_SYSTEM_H
#define INC_PMB_SYSTEM_H

#include "common_defines.h"

#define SYSTICK_FREQ        (1000)      // trigger every 1ms

bool system_setup(void);
uint64_t system_getTicks(void);
void system_delayMs(uint64_t time_ms);

#endif // INC_PMB_SYSTEM_H