/**
 * debug.h - Debug string support
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#ifndef _SRC_DEBUG_H
#define _SRC_DEBUG_H

#include <avr/io.h>
#include <avr/pgmspace.h>

void debug_init();

#ifndef DEBUG_NEWLINE
#define DEBUG_NEWLINE "\r\n"
#endif

#if DEBUG_LEVEL > DEBUG_LEVEL_OFF
#define ERROR(s, ...) printf_P(PSTR("! " __FILE__ ":" __LINE__ ": " s DEBUG_NEWLINE), ## __VA_ARGS__)
#else
#define ERROR(s, ...)
#endif

#if DEBUG_LEVEL > DEBUG_LEVEL_ERROR
#define WARN(s, ...) printf_P(PSTR("W " __FILE__ ":" __LINE__  ": " s DEBUG_NEWLINE), ## __VA_ARGS__)
#else
#define WARN(s, ...)
#endif

#if DEBUG_LEVEL > DEBUG_LEVEL_WARN
#define INFO(s, ...) printf_P(PSTR("I " __FILE__ ":" __LINE__  ": " s DEBUG_NEWLINE), ## __VA_ARGS__)
#else
#define INFO(s, ...)
#endif

#if DEBUG_LEVEL > DEBUG_LEVEL_INFO
#define DEBUG(s, ...) printf_P(PSTR("D " __FILE__ ":" __LINE__  ": " s DEBUG_NEWLINE), ## __VA_ARGS__)
#else
#define DEBUG(s, ...)
#endif

#endif // _SRC_DEBUG_H


