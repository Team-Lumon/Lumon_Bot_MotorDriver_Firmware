#include "queue.h"

#include "main.h"

#include <stdio.h>

static uint32_t queueEnterCritical(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void queueExitCritical(uint32_t primask) {
    if (primask == 0U) {
        __enable_irq();
    }
}

static void printQueueValue(QueueValue_t value) {
    int32_t scaled = (value >= 0.0f) ? (int32_t)(value * 1000.0f + 0.5f)
                                     : (int32_t)(value * 1000.0f - 0.5f);
    uint32_t magnitude = (scaled < 0) ? (uint32_t)(-(int64_t)scaled)
                                      : (uint32_t)scaled;

    if (scaled < 0) {
        printf("-");
    }
    printf("%lu.%03lu", (unsigned long)(magnitude / 1000U),
           (unsigned long)(magnitude % 1000U));
}

void initQueue(Queue *q) {
    if (q == NULL) {
        return;
    }

    uint32_t primask = queueEnterCritical();
    q->front = 0U;
    q->rear = 0U;
    q->count = 0U;
    queueExitCritical(primask);
}

void clearQueue(Queue *q) {
    initQueue(q);
}

bool isEmpty(const Queue *q) {
    if (q == NULL) {
        return true;
    }

    uint32_t primask = queueEnterCritical();
    bool empty = (q->count == 0U);
    queueExitCritical(primask);

    return empty;
}

bool isFull(const Queue *q) {
    if (q == NULL) {
        return false;
    }

    uint32_t primask = queueEnterCritical();
    bool full = (q->count >= QUEUE_CAPACITY);
    queueExitCritical(primask);

    return full;
}

size_t queueSize(const Queue *q) {
    if (q == NULL) {
        return 0U;
    }

    uint32_t primask = queueEnterCritical();
    size_t count = q->count;
    queueExitCritical(primask);

    return count;
}

size_t queueCapacity(void) {
    return QUEUE_CAPACITY;
}

bool enqueue(Queue *q, QueueValue_t value) {
    if (q == NULL) {
        return false;
    }

    uint32_t primask = queueEnterCritical();
    if (q->count >= QUEUE_CAPACITY) {
        queueExitCritical(primask);
        return false;
    }

    q->data[q->rear] = value;
    q->rear = (q->rear + 1U) % QUEUE_CAPACITY;
    q->count++;
    queueExitCritical(primask);

    return true;
}

bool dequeue(Queue *q, QueueValue_t *value) {
    if ((q == NULL) || (value == NULL)) {
        return false;
    }

    uint32_t primask = queueEnterCritical();
    if (q->count == 0U) {
        queueExitCritical(primask);
        return false;
    }

    *value = q->data[q->front];
    q->front = (q->front + 1U) % QUEUE_CAPACITY;
    q->count--;
    queueExitCritical(primask);

    return true;
}

bool peek(const Queue *q, QueueValue_t *value) {
    if ((q == NULL) || (value == NULL)) {
        return false;
    }

    uint32_t primask = queueEnterCritical();
    if (q->count == 0U) {
        queueExitCritical(primask);
        return false;
    }

    *value = q->data[q->front];
    queueExitCritical(primask);

    return true;
}

void printQueue(const Queue *q) {
    QueueValue_t values[QUEUE_CAPACITY];
    size_t count;

    if (q == NULL) {
        printf("Queue: NULL\r\n");
        return;
    }

    /* Take a consistent snapshot, then print with interrupts enabled. */
    uint32_t primask = queueEnterCritical();
    count = q->count;
    for (size_t i = 0U; i < count; i++) {
        values[i] = q->data[(q->front + i) % QUEUE_CAPACITY];
    }
    queueExitCritical(primask);

    printf("Queue (%lu/%lu): [", (unsigned long)count,
           (unsigned long)QUEUE_CAPACITY);
    for (size_t i = 0U; i < count; i++) {
        if (i != 0U) {
            printf(", ");
        }
        printQueueValue(values[i]);
    }
    printf("]\r\n");
}
