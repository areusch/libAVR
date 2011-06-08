/**
 * chip.h - Chip-specific defines to make libAVR work.
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#ifndef _CHIP_H
#define _CHIP_H

#include <avr/io.h>
#include "libavr/chip/util.h"

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
#include "libavr/chip/328p.h"
#else
#error "libAVR not ported to your architecture yet :("
#endif


#endif // _CHIP_H


