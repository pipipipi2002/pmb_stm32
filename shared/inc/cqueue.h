#ifndef INC_CQUEUE_H
#define INC_CQUEUE_H

#include "common_defines.h"

typedef struct {
    uint8_t* data;
    uint32_t size;
    uint32_t head;
    uint32_t tail;
    uint32_t data_max_size;
} cqueue_ts;

void cqueue_init(cqueue_ts* q, uint8_t* buff, uint32_t buff_size);
bool cqueue_isEmpty(cqueue_ts* q);
bool cqueue_isFull(cqueue_ts* q);
uint32_t cqueue_getSize(cqueue_ts* q);
uint8_t cqueue_peek(cqueue_ts* q);
uint8_t cqueue_pushn(cqueue_ts* q, uint8_t* data, uint8_t len);
bool cqueue_push(cqueue_ts* q, uint8_t data);
bool cqueue_pop(cqueue_ts* q, uint8_t* data);
void cqueue_reset(cqueue_ts* q);

#endif // INC_CQUEUE_H