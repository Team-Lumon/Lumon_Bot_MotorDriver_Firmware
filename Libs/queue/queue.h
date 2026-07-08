#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>
#include <stddef.h>

#ifndef QUEUE_CAPACITY
#define QUEUE_CAPACITY 32U
#endif

#if QUEUE_CAPACITY == 0U
#error "QUEUE_CAPACITY must be greater than zero"
#endif

typedef struct Queue {
    int data[QUEUE_CAPACITY];
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

bool enqueue(Queue *q, int value);
bool dequeue(Queue *q, int *value);
bool peek(const Queue *q, int *value);

#endif
