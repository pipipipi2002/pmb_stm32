#ifndef INC_CQUEUE_H
#define INC_CQUEUE_H

#include "common_defines.h"
#include "can_if.h"

typedef struct {
    canFrame_ts* data;
    uint32_t size;
    uint32_t head;
    uint32_t tail;
    uint32_t data_max_size;
} cqueue_ts;

void cqueue_init(cqueue_ts* q, canFrame_ts* buff, uint32_t buff_size);
bool cqueue_isEmpty(cqueue_ts* q);
bool cqueue_isFull(cqueue_ts* q);
uint32_t cqueue_getSize(cqueue_ts* q);
canFrame_ts cqueue_peek(cqueue_ts* q);
bool cqueue_push(cqueue_ts* q, canFrame_ts* data);
bool cqueue_pop(cqueue_ts* q, canFrame_ts* data);

#endif // INC_CQUEUE_H