/** @file usart.c
 *  Hardware USART Services.
 *  Copyright (C) 2010 Andrew Reusch <areusch@gmail.com>
 *
 *  This implementation supports a maximum of 8 USARTs.
 *
 *  Contributions:
 */

#include "usart.h"
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdio.h>
#include "chip.h"
#include "gpio.h"

inline uint16_t compute_bsel(uint32_t baud_rate,
                             int8_t bscale,
                             uint8_t double_pump_scaling) {
  if (bscale >= 0)
    return (((((uint32_t) F_CPU) / baud_rate) >> bscale) >> double_pump_scaling);
  else
    return (((((uint32_t) F_CPU) << (-bscale)) / baud_rate) >> double_pump_scaling) - (1 << (-bscale));
}

inline uint32_t compute_baud(uint16_t bsel,
                             int8_t bscale,
                             uint8_t double_pump_scaling) {
  if (bscale >= 0)
    return ((((uint32_t) F_CPU) / (bsel + 1)) >> bscale) >> double_pump_scaling;
  else
    return ((((uint32_t) F_CPU) << (-bscale)) / (bsel + (1 << (-bscale)))) >> double_pump_scaling;
}

bool usart_init(USART_t* usart,
                uint32_t baud_rate,
                uint8_t usart_mode,
                bool two_stop_bits,
                uint8_t character_size,
                uint8_t parity,
                UsartControl* out_control) {
  if (usart_mode > USART_MODE_SYNCHRONOUS ||
      character_size > 8 ||
      character_size < 5 ||
      parity == 0b01 ||
      parity > USART_PARITY_ODD ||
      !out_control) {
    return false;
  }

  out_control->usart = usart;
  out_control->receive_buffer = NULL;
  out_control->enable_watch_byte = false;
  out_control->ongoing_transfer.buffer = NULL;

  chip_usart_enable(usart, false);

  if (!chip_usart_set_baud(usart, baud_rate, 0))
    return false;

  if (!chip_configure_usart(usart,
                            usart_mode,
                            two_stop_bits,
                            parity,
                            character_size))
    return false;

  gpio_set_output(chip_usart_txd_pin(usart));

  chip_usart_enable(usart, true);
}

void usart_send(UsartControl* usart, uint8_t* data, uint8_t length) {
  while (usart->ongoing_transfer.buffer) ;

  for (uint8_t i = 0; i < length; ++i) {
    while (!(chip_usart_can_send(usart))) ;
    chip_usart_send(usart, data[i]);
  }
}

void usart_send_cstring(UsartControl* usart, const char* str) {
  while (usart->ongoing_transfer.buffer) ;

  for (; *str; ++str) {
    while (!(chip_usart_can_send(usart->usart))) ;
    chip_usart_send(usart->usart, *str);
  }
}

static void usart_recount_watch_bytes(UsartControl* usart) {
  usart->watch_byte_seen_count = 0;
  for (const uint8_t* it = circular_buffer_begin(usart->receive_buffer);
       it != circular_buffer_end(usart->receive_buffer);
       it = circular_buffer_next(usart->receive_buffer, it)) {
    if (*it == usart->watch_byte)
      usart->watch_byte_seen_count++;
  }
}

void usart_set_receive_buffer(UsartControl* usart,
                              CircularBuffer* buf) {
  bool restore_interrupt = false;
  if (chip_usart_is_recv_interrupt_enabled(usart)) {
    restore_interrupt = true;
    cli();
  }

  if (!buf) {
    chip_usart_set_recv_interrupt(usart->usart, false);

    if (restore_interrupt)
      sei();
    return;
  }

  if (usart->receive_buffer != buf &&
      usart->enable_watch_byte) {
    usart->receive_buffer = buf;
    usart_recount_watch_bytes(usart);
  } else {
    usart->receive_buffer = buf;
  }

  if (buf)
    chip_usart_set_recv_interrupt(usart->usart, true);
  if (restore_interrupt)
    sei();
}

void usart_set_buffer_watching(UsartControl* usart,
			       bool enabled,
			       uint8_t watch_byte) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    usart->watch_byte = watch_byte;
    usart->enable_watch_byte = enabled;

    if (usart->receive_buffer && usart->enable_watch_byte)
      usart_recount_watch_bytes(usart);
    else if (!usart->receive_buffer)
      usart->watch_byte_seen_count = 0;
  }
}

bool usart_seen_watch_byte(UsartControl* usart) {
  return usart->watch_byte_seen_count;
}

void usart_receive(UsartControl* usart,
                   uint8_t* receive_buffer,
                   uint8_t buffer_num_bytes) {
  if (usart_receive_buffer_enabled(usart)) {
    CircularBuffer* buf = usart->receive_buffer;
    uint8_t read_bytes = 0;

    while (read_bytes < buffer_num_bytes) {
      while (!circular_buffer_size(buf)) ;
      read_bytes += circular_buffer_read(buf,
                                         receive_buffer + read_bytes,
                                         buffer_num_bytes - read_bytes);
    }

    if (usart->enable_watch_byte && usart->receive_buffer) {
      for (uint8_t i = 0; i < read_bytes; ++i) {
        if (receive_buffer[i] == usart->watch_byte) {
          usart->watch_byte_seen_count--;
        }
      }
    }
  } else {
    for (uint8_t i = 0; i < buffer_num_bytes; ++i) {
      while (!chip_usart_can_recv(usart)) ;
      receive_buffer[i] = chip_usart_recv(usart);
    }
  }
}

void usart_send_async(UsartControl* usart,
                      uint8_t* data,
                      uint16_t length_bytes,
                      usart_callback_fn done) {
  while ((usart->ongoing_transfer.buffer) || !chip_usart_can_send(usart)) ;

  UsartTransfer* xfer = &usart->ongoing_transfer;
  xfer->buffer = data;
  xfer->buffer_size_bytes = length_bytes;
  xfer->current_byte = 0;
  xfer->transfer_complete_fn = done;

  chip_usart_set_xmit_interrupt(usart, true);
}

#if defined(DEBUG_LEVEL) && DEBUG_LEVEL > DEBUG_LEVEL_OFF

USART_t* usart_stdio;

static FILE usart_stdio_stream;

int usart_stdio_stream_put(char a, FILE* stream) {
    while (!(usart_stdio->STATUS & USART_DREIF_bm)) ;
    usart_stdio->DATA = a;
    return 0;
}

int usart_stdio_stream_get(FILE* stream) {
    while (!(usart_stdio->STATUS & USART_RXCIF_bm)) ;
    return usart_stdio->DATA;
}

void usart_set_stdio(USART_t* usart) {
    usart_stdio = usart;
    fdev_setup_stream(&usart_stdio_stream,
                      &usart_stdio_stream_put,
                      &usart_stdio_stream_get,
                      _FDEV_SETUP_RW);
    stderr = &usart_stdio_stream;
    stdout = &usart_stdio_stream;
    stdin = &usart_stdio_stream;
}
#endif
