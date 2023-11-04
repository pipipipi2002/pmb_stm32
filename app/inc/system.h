#ifndef INC_PMB_SYSTEM_H
#define INC_PMB_SYSTEM_H

#define SYSTICK_FREQ        (1000)      // trigger every 1ms

void PMB_system_init(void);
uint64_t PMB_system_getTicks(void);
void PMB_system_delayMs(uint64_t time_ms);

#endif // INC_PMB_SYSTEM_H