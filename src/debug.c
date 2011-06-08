/**
 * debug.c - Debug functions for libAVR
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#include "libavr/debug.h"
#include "libavr/usart.h"

void debug_init() {
#if DEBUG_LEVEL > DEBUG_LEVEL_OFF
  UsartControl temp;

  if (!usart_init(&DEBUG_USART,
                  DEBUG_PORT_BAUD_RATE,
                  USART_MODE_ASYNCHRONOUS,
                  false,
                  8,
                  USART_PARITY_NONE,
                  &temp))
    for (;;) ;

  usart_set_stdio(&DEBUG_USART);
#endif // DEBUG_LEVEL > DEBUG_LEVEL_OFF
}
