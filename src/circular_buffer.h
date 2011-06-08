/** @file circular_buffer.h
 *  Circular Buffering
 *  Copyright (C) 2010 Andrew Reusch <areusch@gmail.com>
 *
 *  Contributions:
 */

#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

#include <inttypes.h>
#include "libavr/assert.h"

#define EXTERNAL_CONST_INTERFACE _CIRCULAR_BUFFER
#include "libavr/external-const.h"

/** State required to maintain a circular buffer.
 */
typedef struct CircularBuffer {
    /** Backing data buffer.
     * This must be supplied by the user to @ref circular_buffer_init(). Its
     * size, in bytes, should be a power of 2.
     */
    EXTERNAL_CONST uint8_t* EXTERNAL_CONST buffer;

    /** Index to the next byte to be read.
     * This value is equal to tail if the circular buffer is empty.
     */
    volatile EXTERNAL_CONST uint8_t head;

    /** Index to the next free byte in the buffer.
     * This value is equal to head if the circular buffer is empty.
     */
    volatile EXTERNAL_CONST uint8_t tail;

    /** Bitmask covering valid values for the head and tail fields.
     * This value is always 1 less than the length of @ref buffer because
     * buffer's size must be a power of 2.
     */
    EXTERNAL_CONST uint8_t buffer_index_bitmask;
} CircularBuffer;

/** Initialize a CircularBuffer for use.
 *
 * Initializes a CircularBuffer to be empty. The size of the backing store, in
 * bytes, must be a power of 2.
 *
 * @param buf User-allocated Buffer struct to be initialized.
 * @param backing_buffer User-allocated backing array. Must be a power of 2
 *                       bytes long.
 * @param buffer_size The length of backing_buffer, in bytes. Must be a power of
 *                    2.
 */
void circular_buffer_init(CircularBuffer* buf,
                          uint8_t* backing_buffer,
                          uint8_t buffer_size);

/** Write @c buffer_num_bytes from @c read_buffer into @c buf.
 *
 * This does not guarantee that @c buffer_num_bytes of data are actually
 * written to @c buf. This may fail because @c buf is full. It is up to the
 * client to block in this case and ensure all bytes in @c read_buffer are
 * successfully written.
 *
 * @param buf CircularBuffer to write
 * @param data array holding the data to write
 * @param data_num_bytes size of data, in bytes.
 * @return the number of bytes actually written to @c buf.
 */
uint8_t circular_buffer_write(CircularBuffer* buf,
                           uint8_t* data,
                           uint8_t data_num_bytes);

/** Read at most @c buffer_num_bytes from @c buf into @c read_buffer.
 *
 * Tries to read up to @c buffer_num_bytes from @c buf. This may fail because
 * @c buf has less data than @c buffer_num_bytes at the time of invocation.
 *
 * @param buf CircularBuffer to read from
 * @param read_buffer array to read data to
 * @param buffer_num_bytes size of @c read_bufer, in bytes.
 * @return the number of bytes actually read.
 */
uint8_t circular_buffer_read(CircularBuffer* buf,
                             uint8_t* read_buffer,
                             uint8_t buffer_num_bytes);

/**
 * Returns a pointer to the start of the circular buffer. Intended
 * for use as with C++-style iterators, except because operator
 * overloading is lacking, use the _begin, _next, _prev, and _end
 * functions.
 *
 * @param buf CircularBuffer to iterate.
 * @return a pointer to the first character in the array.
 */
inline const uint8_t* circular_buffer_begin(CircularBuffer* buf) {
  return &buf->buffer[buf->head];
}

inline const uint8_t* circular_buffer_next(CircularBuffer* buf, const uint8_t* it) {
  if (it - buf->buffer == buf->buffer_index_bitmask)
    return buf->buffer;
  return it + 1;
}

inline const uint8_t* circular_buffer_prev(CircularBuffer* buf, const uint8_t* it) {
  if (it == buf->buffer)
    return buf->buffer + buf->buffer_index_bitmask;
  return it - 1;
}

inline const uint8_t* circular_buffer_end(CircularBuffer* buf) {
  return &buf->buffer[buf->tail];
}

/** Determines the number of bytes possible to store in @c buf.
 *
 * @param buf CircularBuffer to examine
 * @return the number of bytes that @c buf may store at one time.
 */
inline uint8_t circular_buffer_capacity(CircularBuffer* buf) {
    return buf->buffer_index_bitmask + 1;
}

/** Determines the number of bytes in @c buf at the time of invocation.
 *
 * @param buf CircularBuffer to examine
 * @return the number of bytes that @c buf holds at the time of invocation.
 */
inline uint8_t circular_buffer_size(CircularBuffer* buf) {
    if (buf->head >= buf->tail)
        return buf->tail + circular_buffer_capacity(buf) - buf->head - 1;
    else
        return buf->tail - buf->head - 1;
}


inline const uint8_t* circular_buffer_pop(CircularBuffer* buf, const uint8_t* it) {
  ASSERT(it == circular_buffer_begin(buf));

  if (!circular_buffer_size(buf))
    return circular_buffer_end(buf);

  const uint8_t* next = circular_buffer_next(buf, it);
  * ((uint8_t*) &buf->head) = (buf->head + 1) & buf->buffer_index_bitmask;
  return next;
}


#endif /* _CIRCULAR_BUFFER_H_ */
