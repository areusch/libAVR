/**
 * util.h - Utility Routines
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#ifndef _UTIL_H
#define _UTIL_H

#include <inttypes.h>

#define SET_BIT_TO(reg, bit, val) \
  if (val)                                              \
    (reg) |= _BV(bit);                                  \
  else                                                  \
    (reg) &= ~_BV(bit);

#define IS_BIT_SET(reg, bit) ((reg) & _BV(bit))

typedef uint16_t integer_ptr_t;

// Absolute difference between a and b.
#define ABS_DIFF(a, b) ((a) > (b) ? (a) - (b) : (b) - (a))

/** Right-rotate @p value repeatedly by @scale_magnitude binary places
 * until value is beneath or equals @p ceiling. Returns the number of iterations
 * necessary.
 *
 * This is really useful for computing prescaler settings such that the scaled
 * signal is below a threshold. This comes up in timers, serial I/O, oscillator
 * control, and anywhere else clock signals are scaled.
 */
static inline uint8_t scale_by_power_two_until_beneath(uint32_t value,
                                                       uint8_t scale_magnitude,
                                                       uint32_t ceiling) {
  uint8_t iterations;
  for (iterations = 0; value > ceiling; ++iterations)
    value >>= scale_magnitude;

  return iterations;
}

#endif // _UTIL_H
