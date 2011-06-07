/**
 * debug.c - Debug functions for libAVR
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#include "debug.h"

void debug_init() {
#if DEBUG_LEVEL > DEBUG_LEVEL_OFF
  usart_init(&DEBUG_USART,
             DEBUG_PORT_BAUD_RATE,
             false,
             false,
             USART_PARITY_NONE);

  usart_set_stdio(&DEBUG_USART);
#endif // DEBUG_LEVEL > DEBUG_LEVEL_OFF
}
