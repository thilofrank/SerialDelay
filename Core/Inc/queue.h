#ifndef BYTE_QUEUE_H
#define BYTE_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

// Größe der Queue in Bytes
#define QUEUE_SIZE 4048

// Strukturdefinition für die Byte-Queue
typedef struct {
    uint8_t buffer[QUEUE_SIZE];  // Speicherpuffer
    uint16_t head;               // Schreibindex
    uint16_t tail;               // Leseindex
    uint16_t count;              // Anzahl der gespeicherten Bytes
} ByteQueue;

// --- Funktionsprototypen ---

/**
 * @brief   Initialisiert die Queue.
 * @param   q   Zeiger auf ByteQueue-Struktur
 */
void queue_init(ByteQueue *q);

/**
 * @brief   Prüft, ob die Queue leer ist.
 * @param   q   Zeiger auf ByteQueue-Struktur
 * @return  true, wenn leer; sonst false
 */
bool queue_is_empty(const ByteQueue *q);

/**
 * @brief   Prüft, ob die Queue voll ist.
 * @param   q   Zeiger auf ByteQueue-Struktur
 * @return  true, wenn voll; sonst false
 */
bool queue_is_full(const ByteQueue *q);

/**
 * @brief   Fügt ein Byte in die Queue ein.
 * @param   q       Zeiger auf ByteQueue-Struktur
 * @param   data    Das einzufügende Byte
 * @return  true, wenn erfolgreich; false, wenn die Queue voll ist
 */
bool queue_enqueue(ByteQueue *q, uint8_t data);

/**
 * @brief   Entnimmt ein Byte aus der Queue.
 * @param   q       Zeiger auf ByteQueue-Struktur
 * @param   data    Zeiger auf Variable zum Speichern des entnommenen Bytes
 * @return  true, wenn erfolgreich; false, wenn die Queue leer ist
 */
bool queue_dequeue(ByteQueue *q, uint8_t *data);

/**
 * @brief   Liest das nächste Byte, ohne es zu entfernen.
 * @param   q       Zeiger auf ByteQueue-Struktur
 * @param   data    Zeiger auf Variable zum Speichern des gelesenen Bytes
 * @return  true, wenn erfolgreich; false, wenn die Queue leer ist
 */
bool queue_peek(const ByteQueue *q, uint8_t *data);

#endif // BYTE_QUEUE_H
