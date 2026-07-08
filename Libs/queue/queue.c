#include "queue.h"

void initQueue(Queue *q) {
    if (q == NULL) {
        return;
    }

    q->front = 0U;
    q->rear = 0U;
    q->count = 0U;
}

void clearQueue(Queue *q) {
    initQueue(q);
}

bool isEmpty(const Queue *q) {
    return (q == NULL) || (q->count == 0U);
}

bool isFull(const Queue *q) {
    return (q != NULL) && (q->count >= QUEUE_CAPACITY);
}

size_t queueSize(const Queue *q) {
    if (q == NULL) {
        return 0U;
    }

    return q->count;
}

size_t queueCapacity(void) {
    return QUEUE_CAPACITY;
}

bool enqueue(Queue *q, int value) {
    if ((q == NULL) || isFull(q)) {
        return false;
    }

    q->data[q->rear] = value;
    q->rear = (q->rear + 1U) % QUEUE_CAPACITY;
    q->count++;

    return true;
}

bool dequeue(Queue *q, int *value) {
    if ((q == NULL) || (value == NULL) || isEmpty(q)) {
        return false;
    }

    *value = q->data[q->front];
    q->front = (q->front + 1U) % QUEUE_CAPACITY;
    q->count--;

    return true;
}

bool peek(const Queue *q, int *value) {
    if ((q == NULL) || (value == NULL) || isEmpty(q)) {
        return false;
    }

    *value = q->data[q->front];

    return true;
}
