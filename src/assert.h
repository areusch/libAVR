/**
 * assert.h - Assertions for libAVR
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#ifndef _SRC_ASSERT_H
#define _SRC_ASSERT_H

#include <avr/pgmspace.h>
#include <stdio.h>

#ifndef NDEBUG
#define ASSERT(test)                            \
  if (!(test))                                  \
    printf_P(PSTR("ASSERT failed: " __FILE__ ": %d: %s"), __LINE__, # test)
#else
#define ASSERT(test, msg, ...)
#endif

#endif // _SRC_ASSERT_H


