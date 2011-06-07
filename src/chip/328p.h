/**
 * 328p.h - Desc
 * Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 */

#ifndef _CHIP_328P_H
#define _CHIP_328P_H

#include <avr/io.h>
#include <stdbool.h>

// --------------------------------[ ADC Routines ]-----------------------------
//

typedef void ADC_t;
typedef uint16_t adc_result_t;

inline void chip_adc_set_enabled(ADC_t* adc, bool enabled) {
  SET_BIT_TO(ADCSRA, ADEN, enabled);
}

inline void chip_adc_set_reference(ADC_t* adc, uint8_t reference_selection) {
  ADMUX = ADMUX & (_BV(REFS0) - 1) | (reference_selection << REFS0);
}

inline void chip_adc_set_left_adjusted(ADC_t* adc, bool enabled) {
  SET_BIT_TO(ADMUX, ADLAR, enabled);
}

inline void chip_adc_select_input(ADC_t* adc,
                                  uint8_t positive_input,
                                  uint8_t negative_input) {
  ADMUX = (ADMUX & (0xFF - ((_BV(MUX3) << 1) - 1))) | positive_input;
}

inline bool chip_adc_set_minimum_clock(ADC_t* adc, uint32_t minimum_adc_clock) {
  uint8_t prescaler = scale_by_power_two_until_beneath(F_CPU,
                                                       1,
                                                       minimum_adc_clock);
  if (prescaler > 8 || prescaler == 0)
    return false;

  ADCSRA = ADCSRA & (0xFF - ((1 << (ADPS2 + 1)) - 1)) | (prescaler - 1);
  return true;
}

inline void chip_adc_set_interrupt(ADC_t* adc, bool enabled) {
  SET_BIT_TO(ADCSRA, ADIE, enabled);
}

inline void chip_adc_start_conversion(ADC_t* adc) {
  ADCSRA |= _BV(ADSC);
}

inline bool chip_adc_is_conversion_active(ADC_t* adc) {
  return ADCSRA & _BV(ADSC);
}

inline adc_result_t chip_adc_read(ADC_t* adc) {
  return (ADMUX & ADLAR ? (ADCW >> 6) : ADCW);
}

// --------------------------------[ GPIO Routines ]----------------------------
//

#define CHIP_PORT_TO_INDEX_COMPUTATION(port) ((port) << 3)
#define CHIP_INDEX_TO_PORT_COMPUTATION(i)    ((i) >> 3)

#define CHIP_COMMON_GPIO_BITVECTOR_TYPE uint8_t

// Note: DDR register is one byte below PORT register
#define CHIP_GPIO_SET_INPUT(port, pin) (*(port - 1) &= ~(pin))
#define CHIP_GPIO_SET_OUTPUT(port, pin) (*(port - 1) |= (pin))
#define CHIP_GPIO_SET_PULLUPS(port, pin) (MCUCR = MCUCR & ~(1 << PUD) | (pin << PUD))

#define CHIP_GPIO_SET(port, pin, value) (*(port) = (*(port) & ~(1 << pin)) | ((value & 0x01) << pin))
#define CHIP_GPIO_TOGGLE(port, pin) (*(port) = (*(port) ^ (1 << pin)))

// Note: PIN register is 2 bytes below PORT register.
#define CHIP_GPIO_READ(port, pin) (*(port - 2) & (1 << pin))

// 328p uses common GPIO data structures.
#include "chip/gpio.h"

GpioPin chip_adc_get_pin(ADC_t* adc, uint8_t analog_in) {
  GpioPin p = DEFINE_UNNAMED_PIN(PORTC, analog_in);
  return p;
}


// --------------------------------[ SPI Routines ]-----------------------------

typedef void SPI_t;

#define CHIP_NUM_SPI                 1
#define CHIP_SPI_MIN_CLOCK_SPEED_HZ  (F_CPU / 128)

inline void chip_spi_enable(bool enable) {
  SET_BIT_TO(SPCR, SPE, enable);
}

inline void chip_spi_set_interrupt(SPI_t* port, bool enable) {
  SET_BIT_TO(SPCR, SPIE, enable);
}

inline bool chip_spi_is_interrupt_queued(SPI_t* port) {
  return IS_BIT_SET(SPCR, SPIE);
}

inline GpioPin chip_spi_slave_select_pin(SPI_t* port) {
  GpioPin a = DEFINE_UNNAMED_PIN(PORTB, 2);
  return a;
}

inline GpioPin chip_spi_mosi_pin(SPI_t* port) {
  GpioPin a = DEFINE_UNNAMED_PIN(PORTB, 3);
  return a;
}

inline GpioPin chip_spi_miso_pin(SPI_t* port) {
  GpioPin a = DEFINE_UNNAMED_PIN(PORTB, 4);
  return a;
}

inline GpioPin chip_spi_sck_pin(SPI_t* port) {
  GpioPin a = DEFINE_UNNAMED_PIN(PORTB, 5);
  return a;
}

inline bool chip_spi_set_clock_speed(SPI_t* port, uint32_t max_clock_speed_hz) {
  uint8_t clock_divider_setting = scale_by_power_two_until_beneath(F_CPU,
                                                                   4,
                                                                   max_clock_speed_hz);
  if (clock_divider_setting > 7)
    return false;

  // NOTE: ~LSB == CLK2X; bits 1:0 == SPR1:SPR0
  SPSR = (~clock_divider_setting & 0x01);
  SPCR = (SPCR & 0xFC) | (clock_divider_setting >> 1);

  return true;
}

inline void chip_spi_set_operating_params(SPI_t* port,
                                          bool lsb_first,
                                          bool master,
                                          uint8_t spi_mode) {
  SPCR = (SPCR & 0xc3) | (lsb_first << DORD) | (master << MSTR) | (spi_mode & 0x03) << CPHA;
}

inline volatile uint8_t* chip_spi_transfer_register(SPI_t* port) {
  return &SPDR;
}

inline bool chip_spi_set_transfer_interrupt(SPI_t* port, bool enabled) {
  SET_BIT_TO(SPCR, SPIE, enabled);
}

//-------------------------------[ USART Routines ]-----------------------------

#define CHIP_NUM_USARTS 1
typedef void USART_t;

inline GpioPin chip_usart_txd_pin(USART_t* usart) {
  GpioPin a = DEFINE_UNNAMED_PIN(PORTD, 1);
  return a;
}

inline GpioPin chip_usart_rxd_pin(USART_t* usart) {
  GpioPin a = DEFINE_UNNAMED_PIN(PORTD, 0);
  return a;
}

inline void chip_usart_enable(USART_t* usart, bool enable) {
  SET_BIT_TO(UCSR0B, RXEN0, enable);
  SET_BIT_TO(UCSR0B, TXEN0, enable);
}

inline bool chip_usart_set_baud(USART_t* usart,
                                uint32_t baud_rate,
                                uint32_t tolerance_baud_rate) {
  // Baud must be possibly-achievable.
  if ((F_CPU >> 3) < baud_rate)
    return false;

  uint16_t clk2x_ubbr = (F_CPU >> 3) / baud_rate - 1;
  uint32_t clk2x_baud = (F_CPU >> 8) / (clk2x_ubbr + 1);
  uint16_t std_ubbr = (F_CPU >> 4) / baud_rate - 1;
  uint32_t std_baud = (F_CPU >> 4) / (std_baud + 1);

  // Register parameters, used to set register values if we can achieve the
  // baud.
  bool use_2x = false;
  uint16_t ubbr = 0;

  if (((F_CPU >> 4) < baud_rate) ||
      (ABS_DIFF(clk2x_baud, baud_rate) < ABS_DIFF(std_baud, baud_rate))) {
    use_2x = true;
  } else {
    use_2x = false;
  }

  if (use_2x) {
    // Must use 2x prescaler
    if (ABS_DIFF(clk2x_baud, baud_rate) > tolerance_baud_rate)
      return false;

    ubbr = clk2x_ubbr;
  } else {
    if (ABS_DIFF(std_baud, baud_rate) > tolerance_baud_rate)
      return false;

    ubbr = std_ubbr;
  }

  UBRR0 = ubbr;
  SET_BIT_TO(UCSR0A, U2X0, use_2x);

  return true;
}

inline bool chip_configure_usart(USART_t* usart,
                                 uint8_t usart_mode,
                                 bool two_stop_bits,
                                 uint8_t parity,
                                 uint8_t character_size) {
  UCSR0C =
    (usart_mode << UMSEL00) |
    (parity << UPM00) |
    (two_stop_bits ? _BV(USBS0) : 0) |
    (character_size - 5);
  return true;
}

inline bool chip_usart_can_send(USART_t* usart) {
  return IS_BIT_SET(UCSR0A, UDRE0);
}

inline void chip_usart_send(USART_t* usart, uint8_t data) {
  UDR0 = data;
}

inline bool chip_usart_can_recv(USART_t* usart) {
  return IS_BIT_SET(UCSR0A, RXC0);
}

inline uint8_t chip_usart_recv(USART_t* usart) {
  return UDR0;
}

inline bool chip_usart_set_recv_interrupt(USART_t* usart, bool enabled) {
  SET_BIT_TO(UCSR0B, RXCIE0, enabled);
}

inline bool chip_usart_set_xmit_interrupt(USART_t* usart, bool enabled) {
  SET_BIT_TO(UCSR0B, TXCIE0, enabled);
}

inline bool chip_usart_is_recv_interrupt_enabled(USART_t* usart) {
  return IS_BIT_SET(UCSR0B, RXCIE0);
}

#endif // _CHIP_328P_H
