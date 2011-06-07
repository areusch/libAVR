/**
 * adc.c - ADC functions
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#include "adc.h"
#include <stdlib.h>

bool adc_init(ADC_t* adc,
              uint8_t reference_selection,
              bool left_adjust,
              uint32_t minimum_adc_clock) {
  chip_adc_set_enabled(adc, false);
  chip_adc_set_reference(adc, reference_selection);

  chip_adc_set_left_adjusted(adc, left_adjust);

  if (!chip_adc_set_minimum_clock(adc, minimum_adc_clock))
    return false;

  chip_adc_set_interrupt(adc, false);
  chip_adc_set_enabled(adc, true);
  return true;
}

void adc_init_pin(ADC_t* adc, uint8_t analog_in, AnalogInput* out_ain) {
  out_ain->positive_in = analog_in;
  out_ain->positive_pin = chip_adc_get_pin(adc, analog_in);

  // TODO(andrew): Implement!
  out_ain->negative_in = 0;
  GpioPin neg = DEFINE_UNNAMED_PIN((*((void*) 0)), 0);
  out_ain->negative_pin = neg;

  gpio_set_input(out_ain->positive_pin);
}
