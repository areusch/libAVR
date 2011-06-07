/**
 * @file spi.c
 * Flexible hardware SPI Implementation
 * Copyright (C) 2010 Andrew Reusch <areusch@gmail.com>
 *
 * This implementation
 *
 * Contributions:
 */

#include "spi.h"
#include <avr/interrupt.h>
#include <inttypes.h>
#include <stddef.h>
#include "chip.h"
#include "gpio.h"

SpiTransfer** ___spi_active_transfers[CHIP_NUM_SPI];

static bool spi_setup_transfer(SpiTransfer* transfer_request);

bool spi_interrupt_handler(SpiTransfer** transfer,
                           volatile uint8_t* transfer_buffer);

void spi_set_high_impedance(SPI_t* port) {
  gpio_set_input(chip_spi_mosi_pin(port));
  gpio_set_input(chip_spi_sck_pin(port));
  gpio_set_input(chip_spi_miso_pin(port));
}

bool spi_read_miso(SPI_t* spi) {
  return gpio_read(chip_spi_miso_pin(spi));
}

bool spi_init(SPI_t* port,
              bool lsb_first,
              uint8_t spi_mode,
              uint32_t max_clock_speed_hz,
              SpiControl* out_control) {
  if (max_clock_speed_hz < CHIP_SPI_MIN_CLOCK_SPEED_HZ)
        return false;

  chip_spi_enable(false);

  gpio_set_output(chip_spi_mosi_pin(port));
  gpio_set_output(chip_spi_sck_pin(port));
  gpio_set_input(chip_spi_miso_pin(port));

  chip_spi_set_clock_speed(port, max_clock_speed_hz);

  chip_spi_set_operating_params(port, lsb_first, true, spi_mode);

  chip_spi_enable(true);

  out_control->port = port;
  out_control->active_transfer = NULL;

  return true;
}

bool spi_communicate(SpiControl* port, SpiTransfer* transfer_request) {
  // Wait for idle port.
  if (!spi_is_idle(port))
    return false;

  // Bleed off extra data to disable any pending interrupt.
  while (chip_spi_is_interrupt_queued(port))
    *chip_spi_transfer_register(port);

  if (spi_setup_transfer(transfer_request)) {
    chip_spi_set_interrupt(port, true);
    transfer_request->slave->control->active_transfer = transfer_request;
  } else {
    transfer_request->slave->control->active_transfer = NULL;
  }
}

bool spi_setup_transfer(SpiTransfer* rq) {

  if (!rq->acknowledgement_try_count &&
      rq->acknowledgement_mode != kSPINoAcknowledgement) {
    rq->acknowledgement_mode = kSPINoAcknowledgement;
  }

  if (!rq->buffer_size_bytes &&
      rq->acknowledgement_mode == kSPINoAcknowledgement) {
    // No data to transfer; xfer complete.

    if (rq->deselect_slave_when_finished)
      spi_select_slave(rq->slave, false);

    if (rq->transfer_complete_callback)
      rq->transfer_complete_callback(rq);

    return false;
  }

  if (rq->receive_offset < rq->buffer_size_bytes &&
      rq->receive_offset + rq->receive_bytes > rq->buffer_size_bytes)
    rq->receive_bytes = rq->buffer_size_bytes - rq->receive_offset;

  spi_select_slave(rq->slave, true);

  rq->slave->control->current_byte = 0;

  // Start the transfer by writing data. The interrupt will finish things off.
  if (rq->buffer_size_bytes) {
    if (rq->use_transmit_constant) {
      *chip_spi_transfer_register(rq->slave->control->port) = rq->transmit_constant;
    } else {
      *chip_spi_transfer_register(rq->slave->control->port) =
        ((uint8_t*) rq->transfer_buffer)[rq->slave->control->current_byte];
    }
  } else {
    *chip_spi_transfer_register(rq->slave->control->port) = rq->acknowledgement_request_byte;
  }
  return true;
}

bool spi_is_idle(SpiControl* control) {
  return control->active_transfer;
}

void spi_terminate_current_transfer(SpiControl* control) {
  // No need for atomicity. Be a benevolent SPI dictator, and try to let
  // transfers finish if we can.
  if (control->active_transfer) {
    chip_spi_set_transfer_interrupt(control->port, false);
    control->active_transfer = NULL;
  }
}

bool spi_interrupt_handler(SpiTransfer** transfer_ptr,
                           volatile uint8_t* transfer_buffer) {

  SpiTransfer* xfer = *transfer_ptr;

  if (xfer->slave->control->current_byte >= xfer->receive_offset &&
      xfer->slave->control->current_byte < xfer->receive_offset + xfer->receive_bytes) {
    xfer->transfer_buffer[xfer->slave->control->current_byte] = *transfer_buffer;
  }

  // At this point we've finished receiving a byte and need to go on to
  // transmitting the next one.
  xfer->slave->control->current_byte++;

  if (xfer->slave->control->current_byte < xfer->buffer_size_bytes) {
    if (xfer->use_transmit_constant) {
      *transfer_buffer = xfer->transmit_constant;
    } else {
      *transfer_buffer = xfer->transfer_buffer[xfer->slave->control->current_byte];
    }
  } else {
    bool transfer_complete = false;

    // Last byte was transferred. See if an acknowledgement is needed.
    // If we are waiting for an acknowledgement, *current_byte
    // will larger than buffer_size_bytes.

    // Number of times we have sent a byte looking for an acknowledgement.
    uint16_t ack_num_tries = xfer->slave->control->current_byte - xfer->buffer_size_bytes;
    if (ack_num_tries) {
      // We are waiting for an acknowledgement. We won't have read the transfer
      // buffer yet.
      uint8_t spi_received_byte = *transfer_buffer;

      if (ack_num_tries <= xfer->acknowledgement_try_count) {
        switch (xfer->acknowledgement_mode) {
        case kSPIWaitForAckByte:
          if (spi_received_byte == xfer->acknowledgement_byte) {
            xfer->acknowledgement_received = true;
            transfer_complete = true;
          }
          break;
        case kSPIWaitForDifferentAck:
          if (spi_received_byte != xfer->acknowledgement_byte) {
            xfer->acknowledgement_received = true;
            transfer_complete = true;
          }
          break;
        default:
          // This is an error condition. End the transfer, failing.
          xfer->acknowledgement_received = false;
          transfer_complete = true;
        }

        if (!transfer_complete &&
            ack_num_tries == xfer->acknowledgement_try_count) {
          transfer_complete = true;
          xfer->acknowledgement_received = false;
        }

        if (transfer_complete) {
          // Received an acknowledgement byte. Save & finish the transfer.
          xfer->acknowledgement_byte = spi_received_byte;
          xfer->acknowledgement_mode = kSPINoAcknowledgement;
        } else {
          // Didn't receive the acknowledgement byte. Continue trying.
          *transfer_buffer = xfer->acknowledgement_request_byte;
        }
      } else {
        // Exceeded acknowledgement_try_count
        xfer->acknowledgement_received = false;
        xfer->acknowledgement_byte = *transfer_buffer;
        transfer_complete = true;
      }

    } else {
      // The transfer has just completed. We need to see if we need to
      // wait for an acknowledgement byte or just finish out the
      // transfer.

      if (xfer->acknowledgement_mode != kSPINoAcknowledgement) {
        *transfer_buffer = xfer->acknowledgement_request_byte;

        // Unlike the previous case, let the current byte counter
        // increment here. We need it to be larger than transfer_buffer
        // on the next go round.
      } else {
        transfer_complete = true;
      }
    }

    if (transfer_complete) {
      if (xfer->deselect_slave_when_finished) {
        spi_select_slave(xfer->slave, false);
      }

      *transfer_ptr = NULL;
      xfer->slave->control->active_transfer = NULL;
      if (xfer->transfer_complete_callback)
        xfer->transfer_complete_callback(xfer);

      return *transfer_ptr != NULL;
    }
  }
  return true;
}
