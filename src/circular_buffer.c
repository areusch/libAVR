/** @file circular-buffer.c
 *  Circular Buffering
 *  Copyright (C) 2010 Andrew Reusch
 *
 *  Contributions:
 */

#define _CIRCULAR_BUFFER 0xEC
#include "libavr/circular_buffer.h"

void circular_buffer_init(CircularBuffer* buf,
                          uint8_t* backing_buffer,
                          uint8_t buffer_size_bytes) {
    buf->buffer = backing_buffer;
    buf->head = 0;
    buf->tail = 1;
    buf->buffer_index_bitmask = buffer_size_bytes - 1;
}

uint8_t circular_buffer_write(CircularBuffer* buf,
                              uint8_t* data,
                              uint8_t data_num_bytes) {
    uint8_t i = 0;
    for (; i < data_num_bytes && buf->tail != buf->head; ++i) {
        buf->buffer[buf->tail] = data[i];
        buf->tail = (buf->tail + 1) & buf->buffer_index_bitmask;
    }
    return i;
}

uint8_t circular_buffer_read(CircularBuffer* buf,
                             uint8_t* read_buffer,
                             uint8_t buffer_num_bytes) {
    uint8_t i = 0;
    for (; i < buffer_num_bytes && ((buf->head + 1) & buf->buffer_index_bitmask) != buf->tail; ++i) {
        buf->head = (buf->head + 1) & buf->buffer_index_bitmask;
        read_buffer[i] = buf->buffer[buf->head];
    }
    return i;
}
