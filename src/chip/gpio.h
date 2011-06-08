/**
 * gpio.h - Desc
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#ifndef _CHIP_GPIO_H
#define _CHIP_GPIO_H

#include <avr/io.h>
#include <inttypes.h>

#if defined(CHIP_USE_COMMON_GPIO) && (!defined(CHIP_PORT_TO_INDEX_COMPUTATION) || !defined(CHIP_INDEX_TO_PORT_COMPUTATION) || !defined(CHIP_COMMON_GPIO_BITVECTOR_TYPE))
#error "To use common GPIO facilities, define all bit-packing parameters"
#endif

typedef struct GpioPin {
  // Pin is a bitmap as follows:
  // n     n - 1     n - 2 ... 3     2    1    0
  // Pn-2  Pn-3      Pn-3      P0    B2   B1   B0
  // P[n:3] indexes the port
  // B[2:0] indexes the bit
  CHIP_COMMON_GPIO_BITVECTOR_TYPE pin;
} GpioPin;

// Null pin; you should be able to pass a GpioPin whose memory is equivalent to
// all 0's to indicate the "NULL" or non-existent pin.
#define null_PORT NULL
#define null_PIN  0

// Generic pin bitvector manipulation routines
#define UNNAMED_PIN_PORT(port)   ((PORT_t*) CHIP_INDEX_TO_PORT_COMPUTATION((port)->pin))
#define UNNAMED_PIN_BIT(port)    ((uint8_t) (1 << ((port)->pin & 0x07)))
#define UNNAMED_PIN_INDEX(port)  ((uint8_t) ((port)->pin & 0x07)

#define DEFINE_UNNAMED_PIN(port, pin) {                                 \
  (CHIP_COMMON_GPIO_BITVECTOR_TYPE) CHIP_PORT_TO_INDEX_COMPUTATION((integer_ptr_t) &port | (pin)) \
  }

typedef volatile uint8_t PORT_t;

#endif // _CHIP_GPIO_H


