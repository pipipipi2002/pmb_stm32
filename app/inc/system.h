#ifndef INC_SYSTEM_H
#define INC_SYSTEM_H

#define SYSTICK_FREQ        (1000)      // trigger every 1ms

void system_init(void);
uint64_t system_getTicks(void);
void system_delay_ms(uint64_t time_ms);

#endif // INC_SYSTEM_H