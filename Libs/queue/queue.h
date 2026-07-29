#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef QUEUE_CAPACITY
#define QUEUE_CAPACITY 1024U
#endif

#if QUEUE_CAPACITY == 0U
#error "QUEUE_CAPACITY must be greater than zero"
#endif

typedef float QueueValue_t;

typedef struct Queue {
    QueueValue_t data[QUEUE_CAPACITY];
    size_t front;
    size_t rear;
    size_t count;
} Queue;

void initQueue(Queue *q);
void clearQueue(Queue *q);

bool isEmpty(const Queue *q);
bool isFull(const Queue *q);
size_t queueSize(const Queue *q);
size_t queueCapacity(void);

bool enqueue(Queue *q, QueueValue_t value);
bool dequeue(Queue *q, QueueValue_t *value);
bool peek(const Queue *q, QueueValue_t *value);
void printQueue(const Queue *q);

#endif
