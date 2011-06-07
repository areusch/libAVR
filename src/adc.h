/**
 * adc.h - Analog to Digital Conversion
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#ifndef _SRC_ADC_H
#define _SRC_ADC_H

#include <stdbool.h>
#include "chip.h"
#include "gpio.h"

typedef struct AnalogInput {
  GpioPin positive_pin;
  uint8_t positive_in;

  GpioPin negative_pin;
  uint8_t negative_in;
} AnalogInput;

bool adc_init(ADC_t* adc,
              uint8_t reference_selection,
              bool left_adjust,
              uint32_t minimum_adc_clock);

void adc_init_pin(ADC_t* adc, uint8_t analog_in, AnalogInput* out_ain);

inline void adc_select(ADC_t* adc, AnalogInput* input) {
  chip_adc_select_input(adc, input->positive_in, input->negative_in);
}

inline void adc_start_conversion(ADC_t* adc) {
  chip_adc_start_conversion(adc);
}

inline bool adc_is_conversion_active(ADC_t* adc) {
  return chip_adc_is_conversion_active(adc);
}

inline adc_result_t adc_read(ADC_t* adc) {
  return chip_adc_read(adc);
}

inline adc_result_t adc_select_and_convert(ADC_t* adc, AnalogInput* input) {
  adc_select(adc, input);
  adc_start_conversion(adc);

  while (adc_is_conversion_active(adc)) ;

  return adc_read(adc);
}

#endif // _SRC_ADC_H

