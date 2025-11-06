#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "queue.h"

// Initialisieren
void queue_init(ByteQueue *q) {
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

// Prüfen, ob leer
bool queue_is_empty(const ByteQueue *q) {
    return q->count == 0;
}

// Prüfen, ob voll
bool queue_is_full(const ByteQueue *q) {
    return q->count == QUEUE_SIZE;
}

// Byte hinzufügen
bool queue_enqueue(ByteQueue *q, uint8_t data) {
    if (queue_is_full(q)) {
        return false;  // Kein Platz
    }
    q->buffer[q->head] = data;
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count++;
    return true;
}

// Byte entnehmen
bool queue_dequeue(ByteQueue *q, uint8_t *data) {
    if (queue_is_empty(q)) {
        return false;  // Nichts zum Lesen
    }
    *data = q->buffer[q->tail];
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count--;
    return true;
}

// Peek (nur ansehen, nicht löschen)
bool queue_peek(const ByteQueue *q, uint8_t *data) {
    if (queue_is_empty(q)) {
        return false;
    }
    *data = q->buffer[q->tail];
    return true;
}

