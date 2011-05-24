/**
 * test.c - Desc
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#include "chip.h"
#include "gpio.h"

#define test_PORT  PORTB
#define test_PIN   6

void do_test() {
  OUTPUT(test);
}

void do_test2(GpioPin pin) {
  gpio_set_output(pin);
}
