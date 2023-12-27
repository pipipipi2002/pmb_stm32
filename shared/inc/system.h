#ifndef INC_PMB_SYSTEM_H
#define INC_PMB_SYSTEM_H

#include "common_defines.h"

#define SYSTICK_FREQ        (1000)      // trigger every 1ms

typedef struct {
    uint64_t timeout;
    uint64_t targetTime;
    bool autoReset;
    bool hasElapsed;
} timeout_ts;

void system_rccInit(void);
void system_systickInit(void);
void system_systickDeinit(void);
uint64_t system_getTicks(void);
void system_delayMs(uint64_t time_ms);

void timeout_setup(timeout_ts* time, uint64_t timeout_ms, bool autoReset);
bool timeout_hasElapsed(timeout_ts* time);
void timeout_turnOff(timeout_ts* time);
void timeout_turnOn(timeout_ts* time);

#endif // INC_PMB_SYSTEM_H