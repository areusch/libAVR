/** @file usart.h
 *  Hardware USART Services.
 *  Copyright (C) 2011 Andrew Reusch <areusch@gmail.com>
 *
 *  This file provides hardware USART services.
 *
 *  This implementation provides synchronous and asynchronous (i.e. interrupt-driven)
 *  operation. It also implements receive buffering (using a backing circular buffer)
 *  and can watch the incoming stream for a sentinel byte.
 *
 *
 *  Contributions:
 */

#ifndef _USART_H_
#define _USART_H_

#include <avr/io.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include "circular_buffer.h"
#include "chip.h"

/**
 * USART Callback for asynchronous transfers.
 * Define a function of this prototype to receive an interrupt-driven callback
 * when a DMA transfer completes. Your function should be limited to setting
 * some global state or making some quick non-blocking I/O. Do not call any
 * scheduler, DMA, or USART functions from your callback, for starters.
 */
typedef void(*usart_callback_fn)();

/**
 * USART Parity Settings.
 * These settings indicate the type of parity supported by the USART hardware.
 */
enum USART_PARITY {
    USART_PARITY_NONE = 0,    /** Specify no parity. */
    USART_PARITY_EVEN = 0b10, /** Specify even parity. */
    USART_PARITY_ODD = 0b11   /** Specify odd parity. */
};

/**
 * USART Mode.
 * Mode of operation for the USART.
 */
enum USART_MODE {
    USART_MODE_ASYNCHRONOUS = 0, /* Asynchronous (i.e. time-based) operation */
    USART_MODE_SYNCHRONOUS = 1,  /* Synchronous (i.e. clocked) operation */
};

typedef struct UsartTransfer {
    volatile uint8_t* buffer;
    uint8_t buffer_size_bytes;
    uint8_t current_byte;
    usart_callback_fn transfer_complete_fn;
} UsartTransfer;

typedef struct UsartControl {
  USART_t* usart;

  CircularBuffer* receive_buffer;

  bool enable_watch_byte;
  uint8_t watch_byte;
  uint8_t watch_byte_seen_count;

  UsartTransfer ongoing_transfer;
} UsartControl;

/**
 * Initialize a USART with the given baud.
 *
 * Call this function before calling any other USART-related functions to set up
 * a USART for communication.
 *
 * This function exhaustively searches all possible baud rate configurations to
 * configure the USART as close to the requested baud as possible.
 *
 * @param usart Address of the USART_t struct for the USART you want to use.
 * @param baud_rate Baud rate to use.
 * @param usart_mode One of USART_MODE describing the USART's mode of operation.
 * @param two_stop_bits true to use two stop bits; false to use one stop bit.
 * @param character_size size, in bits, of one character. 9 bit not supported.
 * @param parity type of parity to use
 * @param out_control caller-owned control structure to set up. Use this for
 *                    all future calls.
 * @return true if USART initialization succeeds.
 */
bool usart_init(USART_t* usart,
                uint32_t baud_rate,
                uint8_t usart_mode,
                bool two_stop_bits,
                uint8_t character_size,
                uint8_t parity,
                UsartControl* out_control);

/**
 * Syncrhonously send data over the USART.
 *
 * This function synchronously sends @c length bytes starting at @c data over
 * the USART identified by @usart. If a DMA transfer is in progress, it blocks
 * the current Task until that transfer completes.
 */
void usart_send(UsartControl* usart, uint8_t* data, uint8_t length);

/**
 * Synchronously send the given C-string out over the USART.
 *
 * This convenience function sends a NULL-terminated string out over
 * the indicated USART.
 *
 * @param usart USART to write
 * @param str String to send
 */
void usart_send_cstring(UsartControl* usart, const char* str);

/**
 * Determine if a transmission is active. Errs toward false positives
 * (i.e. returning true when a transmission may in fact not be active).
 *
 * @param usart USART to examine
 * @return true if a transmission is active
 */
inline bool usart_is_transmission_active(UsartControl* control) {
  return !!control->ongoing_transfer.buffer;
}

/**
 * Transmit the given data buffer using hardware interrupts, and call the
 * specified callback when the transmission completes. This operates corectly
 * when CTS mode is enabled.
 *
 * To ensure correct transmission, the client should ensure that the data buffer
 * is not changed until @p done is called.
 *
 * @param usart USART used for transmission.
 * @param data pointer to the start of the data being transmitted.
 * @param length_bytes number of bytes to transmit from data.
 * @param done pointer to a function to call when the transmission completes.
 */
void usart_send_async(UsartControl* usart,
                      uint8_t* data,
                      uint16_t length_bytes,
                      usart_callback_fn done);

/**
 * Receive data over a USART.
 *
 * This function receives @c buffer_num_bytes of data over @usart into the
 * memory buffer pointed to by @c receive_buffer. @c receive_buffer must be
 * large enough to contain @c buffer_num_bytes bytes.
 *
 * This function blocks the current Task until data is available.
 *
 * @param usart USART to receive data over.
 * @param receive_buffer Buffer to hold received data.
 * @param buffer_num_bytes Maximum number of bytes to read.
 * @return the number of bytes read, which can differ from @p buffer_num_bytes
 *         if a receive buffer is enabled on @p usart.
 */
uint8_t usart_receive(UsartControl* usart,
                      uint8_t* receive_buffer,
                      uint8_t buffer_num_bytes);

/**
 * Convenience function to synchronously receive a byte from the USART.
 *
 * @return byte received from the USART.
 */
inline uint8_t usart_receive_byte(UsartControl* usart) {
    uint8_t received_byte;
    usart_receive(usart, &received_byte, 1);
    return received_byte;
}

/**
 * If using buffered input, checks to see if the receive buffer is empty.  If
 * not using buffered input for that USART, returns false.
 *
 * @param usart USART to inquire on the receive buffer's status.
 * @return true if a receive buffer exists and data can be read.
 */
bool usart_receive_buffer_empty(UsartControl* usart) {
  return !usart->receive_buffer &&
    circular_buffer_size(usart->receive_buffer) > 0;
}

/**
 * Determine if receive buffering is enabled.
 *
 * @param usart USART to examine
 * @return true if receive buffering is enabled, false otherwise.
 */
inline bool usart_receive_buffer_enabled(UsartControl* usart) {
  return !!usart->receive_buffer;
}

/**
 * Sets a USART to use buffered input mode with the given buffer.
 * Receive buffering expects YOU to allocate the receive buffer and maintain
 * it throughout the period in which you'd like to use it.
 *
 * This function will initialize the given UsartBuffer and begin using it
 * when it returns. Make sure that you globally allocate that receive buffer,
 * not declare it in a calling function, or at least call
 * @ref usart_disable_receive_buffer before the calling function returns.
 *
 * @param usart USART to enable receive buffering on.
 * @param buf An uninitialized receive buffering struct
 *
 */
void usart_set_receive_buffer(UsartControl* usart,
                              CircularBuffer* buf);

// ------------------------------[ Buffer Watching ]----------------------------
//
// Buffer watching is somewhat advanced functionality and works only when receive
// buffering is enabled. In this case programs typically wish to operate on
// messages which are delimited either by placing a length indicator at the start
// of the message or by using a pre-specified delimiter. Buffer watching helps with
// the latter case by examining received data from the USART. When a specified byte
// arrives, a refcount will increment; when it is read using usart_read, the
// refcount will decrement. Thus, buffer watching allows user code to determine
// the number of watched characters present in the receive buffer.

/**
 * Set the watch byte on a particular USART, and whether to watch for that byte.
 * Sentinel watching is only enabled when receive buffering is enabled, since
 * otherwise this would require reading the DATA register and losing received data.
 *
 * @param usart USART to affect
 * @param enabled true to enable byte watching, false to disable
 * @param watch_byte byte to watch for
 */
void usart_set_buffer_watching(UsartControl* usart, bool enabled, uint8_t watch_byte);

/**
 * Returns true if the watch byte has been received on the given
 * USART. Returns false if buffer watching is disabled. The watch byte
 * is set using @ref usart_set_buffer_watching.
 *
 * @param usart USART to examine
 * @return the number of watch bytes seen.
 */
uint8_t usart_num_watch_bytes_seen(UsartControl* usart);

/**
 * Link stdout/in/err to a usart.
 * This function allows you to use printf, scanf, and any other functions
 * in avr-libc that use stdout/stderr/stdin. Do not call this function while
 * any functions are using stdout/err/in.
 *
 * Be aware that this may cause scanf and the like to block the current
 * Task until incoming data is available.
 *
 * @param usart USART to use for stdio.
 */
void usart_set_stdio(UsartControl* usart);

/**
 * USART Interrupt handler.
 *
 * This function handles an interrupt either from the USART DRE. Calling this
 * handler implies that one byte of data should be transmitted, though it will
 * still check to ensure that it is safe to do so.
 *
 * This handler will transmit one byte of data if possible and then enable the
 * appropriate interrupt handler which should indicate that it is safe to send
 * another byte.
 */
inline void usart_transmit_interrupt_handler(UsartControl* usart) {
  if (!usart->ongoing_transfer.buffer) {
    chip_usart_set_xmit_interrupt(usart, false);
    return;
  }

  UsartTransfer* xfer = &usart->ongoing_transfer;
  if (xfer->current_byte >= xfer->buffer_size_bytes) {
    xfer->buffer = NULL;
    chip_usart_set_xmit_interrupt(usart, false);
    return;
  }

  chip_usart_send(usart, xfer->buffer[xfer->current_byte++]);
}

/**
 * USART receive handler. Call from an interrupt handler to process an interrupt
 * request on the given USART.
 *
 * @param usart Control structure currently associated with the USART.
 */
inline void usart_receive_interrupt_handler(UsartControl* usart) {
  if (!usart->receive_buffer) {
    chip_usart_set_recv_interrupt(usart, false);
    return;
  }

  uint8_t data = chip_usart_recv(usart);
  if (usart->enable_watch_byte && data == usart->watch_byte)
    usart->watch_byte_seen_count++;

  circular_buffer_write(usart->receive_buffer, (uint8_t*) &data, 1);
}

#endif /* _USART_H_ */
