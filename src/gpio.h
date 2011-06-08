/** @file gpio.h
 *  Macros for GPIO.
 *  Copyright (C) 2010 Andrew Reusch <areusch@gmail.com>
 *
 *  Contributions:
 */

#ifndef _UTIL_GPIO_H_
#define _UTIL_GPIO_H_

#include <avr/io.h>
#include <stdbool.h>
#include "libavr/chip.h"

#define NAMED_PIN_PORT(id) &(id ## _PORT)
#define NAMED_PIN_BIT(id) (1 << (id ## _PIN))

#define DEFINE_PIN(pin) DEFINE_UNNAMED_PIN(NAMED_PIN_PORT(pin), NAMED_PIN_BIT(pin))

#define INPUT(pin) CHIP_GPIO_SET_INPUT(NAMED_PIN_PORT(pin), NAMED_NAMED_PIN_BIT(pin))
#define OUTPUT(pin) CHIP_GPIO_SET_OUTPUT(NAMED_PIN_PORT(pin), NAMED_PIN_BIT(pin))

#define TOGGLE(pin) CHIP_GPIO_TOGGLE(NAMED_PIN_PORT(pin), NAMED_PIN_BIT(pin))
#define HIGH(pin) CHIP_GPIO_SET(NAMED_PIN_PORT(pin), NAMED_PIN_BIT(pin), 1)
#define LOW(pin) CHIP_GPIO_SET(NAMED_PIN_PORT(pin), NAMED_PIN_BIT(pin), 0)

#define VALUE_OF(pin) CHIP_GPIO_READ(NAMED_PIN_PORT(pin), NAMED_PIN_BIT(pin))

inline void gpio_set_input(GpioPin pin) {
  CHIP_GPIO_SET_INPUT(UNNAMED_PIN_PORT(&pin), UNNAMED_PIN_BIT(&pin));
}

inline void gpio_set_output(GpioPin pin) {
  CHIP_GPIO_SET_OUTPUT(UNNAMED_PIN_PORT(&pin), UNNAMED_PIN_BIT(&pin));
}

inline void gpio_toggle(GpioPin pin) {
  CHIP_GPIO_TOGGLE(UNNAMED_PIN_PORT(&pin), UNNAMED_PIN_BIT(&pin));
}

inline void gpio_high(GpioPin pin) {
  CHIP_GPIO_SET(UNNAMED_PIN_PORT(&pin), UNNAMED_PIN_BIT(&pin), 1);
}

inline void gpio_low(GpioPin pin) {
  CHIP_GPIO_SET(UNNAMED_PIN_PORT(&pin), UNNAMED_PIN_BIT(&pin), 0);
}

inline void gpio_set(GpioPin pin, bool set_high) {
  CHIP_GPIO_SET(UNNAMED_PIN_PORT(&pin), UNNAMED_PIN_BIT(&pin), set_high);
}

inline bool gpio_read(GpioPin pin) {
  return CHIP_GPIO_READ(UNNAMED_PIN_PORT(&pin), UNNAMED_PIN_BIT(&pin));
}

#endif /* _UTIL_GPIO_H_ */
