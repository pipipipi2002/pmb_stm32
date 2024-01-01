#include "cqueue.h"

/**
 * @brief Initialise circular queue
 * 
 * @param q Pointer to queue struct
 * @param buff Pointer to supplied buffer
 * @param buff_size Buffer size supplied
 */
void cqueue_init(cqueue_ts* q, uint8_t* buff, uint32_t buff_size) {
    q->data = buff;
    memset(q->data, 0xFF, buff_size);
    q->data_max_size = buff_size;
    q->head = 0;
    q->tail = 0;
    q->size = 0;
}

/**
 * @brief Check whether queue is empty
 * 
 * @param q pointer to queue
 * @return true if queue is empty
 */
bool cqueue_isEmpty(cqueue_ts* q) {
    return (q->size == 0);
}

/**
 * @brief Check whether queue is full
 * 
 * @param q pointer to queue
 * @return true if queue is full
 */
bool cqueue_isFull(cqueue_ts* q) {
    return (q->size == q->data_max_size);
}

/**
 * @brief Get the current size of the queue
 * 
 * @param q pointer to queue
 * @return uint32_t size of the queue
 */
uint32_t cqueue_getSize(cqueue_ts* q) {
    return q->size;
}

/**
 * @brief Get the data frame at the top of the queue 
 * 
 * @param q pointer to queue
 * @return uint8_t data at the top
 */
uint8_t cqueue_peek(cqueue_ts* q) {
    return q->data[q->head];
}

/**
 * @brief Push n frame of data into the queue
 * 
 * @param q pointer to queue
 * @param data data byte array to push
 * @param len length of data byte to push
 * @return uint8_t number of data byte pushed
 */
uint8_t cqueue_pushn(cqueue_ts* q, uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (!cqueue_push(q, data[i])) {
            return i;
        }
    }
    return len;
}

/**
 * @brief Push 1 frame of data into the queue
 * 
 * @param q pointer to queue
 * @param data data byte to push
 * @return true if successful
 * @return false if queue is full
 */
bool cqueue_push(cqueue_ts* q, uint8_t data) {
    if (cqueue_isFull(q)) {
        return false;
    }

    q->data[q->tail++] = data;
    q->tail %= q->data_max_size;
    q->size++;
    return true;
}

/**
 * @brief Remove and return 1 frame of data
 * 
 * @param q pointer to the queue
 * @param data pointer to the data byte
 * @return true if successful
 * @return false if queue is empty
 */
bool cqueue_pop(cqueue_ts* q, uint8_t* data) {
    if (cqueue_isEmpty(q)) {
        return false;
    }

    *data = q->data[q->head++];
    q->head %= q->data_max_size;
    q->size--;

    return true;
}

/**
 * @brief Clear the buffer to 0xFF and reset the queue supporting data
 * 
 * @param q pointer to queue
 */
void cqueue_reset(cqueue_ts* q) {
    memset(q->data, 0xFF, q->data_max_size);
    q->head = 0;
    q->tail = 0;
    q->size = 0;
}