/**
 * util.h - Utility Routines
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#ifndef _UTIL_H
#define _UTIL_H

#define SET_BIT_TO(reg, bit, val) ((reg) = ((reg) & ~(bit)) | ((val) << (bit)))
#define IS_BIT_SET(reg, bit) ((reg) & (1 << (bit)))

#endif // _UTIL_H


