/** @file spi.h
 *  Pure SPI Mode Driver
 *  Copyright (C) 2010 Andrew Reusch <areusch@gmail.com>
 *
 *  TODO: Document.
 *
 *  Contributions:
 */

#ifndef _SPI_H_
#define _SPI_H_

#include <avr/io.h>
#include <inttypes.h>
#include <stdbool.h>
#include "libavr/chip.h"
#include "libavr/gpio.h"

/**
 * SPI Bus Mode.
 * This mode reflects the timing of sampling and setup as well as the clock
 * polarity on the SPI bus.
 */
typedef enum SpiMode {
    kSPIModeSampleRisingSetupFalling = 0,
    kSPIModeSetupRisingSampleFalling = 1,
    kSPIModeSampleFallingSetupRising = 2,
    kSPIModeSetupFallingSampleRising = 3
} SpiMode;

/**
 * SPI Acknowledgement Mode.
 *
 * @see SpiTransfer.acknowledgement_mode
 */
typedef enum SpiAcknowledgementMode {
    kSPINoAcknowledgement = 0,
    kSPIWaitForAckByte = 1, /* Wait for @ref acknowledgement_byte */
    kSPIWaitForDifferentAck = 2  /* Wait for a byte != @ref acknowledgement_byte */
} SpiAcknowledgementMode;

// Forward declaration so compiler doesn't complain about the pointer below.
struct SpiTransfer;

/**
 * SPI Control structure. An initialized SPI port should have an associated
 * control structure.
 */
typedef struct SpiControl {
  SPI_t* port;
  struct SpiTransfer* active_transfer;
  uint16_t current_byte;
} SpiControl;

/**
 * SPI Slave descriptor. Contains a control structure and a slave-select pin.
 */
typedef struct SpiSlave {
  /**
   * Control structure that the slave uses.
   */
  SpiControl* control;

  /**
   * The pin to which the slave is tied.
   */
  GpioPin ss;

  /**
   * True if SS is active-high; false if SS is active-low.
   */
  bool ss_active_high;
} SpiSlave;

/**
 * SPI Callback function. You can initiate SPI transfers from this callback.
 */
typedef void(*spi_callback_fn)(struct SpiTransfer* transfer);

/**
 * Encapsulates a transfer request.
 */
typedef struct SpiTransfer {
  /**
   * Slave to which this transfer is tied.
   */
  SpiSlave* slave;

  /**
   * Data buffer to hold transmitted and received bytes.
   * This buffer is read-only unless receive_bytes is nonzero.
   */
  uint8_t* transfer_buffer;

  /**
   * Number of bytes to transfer and receive.
   * This is the number of bytes that @c transfer_buffer is expected to hold.
   * The transfer operation represented by this data structure will physically
   * transmit and receive this many bytes. You may choose to ignore some or
   * all of the received bytes using @c receive_bytes and @c receive_offset.
   */
  uint16_t buffer_size_bytes;

  /**
   * If true, transmit @ref transmit_constant instead of transfer_buffer.
   */
  bool use_transmit_constant;

  /**
   * Constant value to transmit.
   * If @ref use_transmit_constant_value is true, transmit this byte @ref
   * buffer_size_bytes times.
   */
  uint8_t transmit_constant;

  /**
   * Offset to start recording received bytes.
   * If @c receive_bytes is nonzero, the SPI driver will start saving
   * received bytes inside of @c transfer_buffer after this many bytes have
   * been transmitted. This value must be between 0 and buffer_size_bytes - 1
   * inclusive if @c receive_bytes is nonzero.
   *
   * The received data will be between @c transfer_buffer[receive_offset] and
   * @c transfer_buffer[receive_offset + receive_bytes - 1]
   */
  uint16_t receive_offset;

  /**
   * Number of received bytes to record in @c transfer_buffer.
   * Per the description of @c receive_offset, this many received bytes will
   * be recorded in @c transfer_buffer.
   */
  uint16_t receive_bytes;

  /**
   * Allow the slave device to signal the end/completeness of transfer.
   * If not SPI_NO_ACKNOWLEDGEMENT, a set of criteria are established to
   * determine if the slave has successfully acknowledged the transfer. Those
   * criteria depend on the value of this field, and the choices of criteria
   * are enumerated below with the corresponding setting of this field.
   *
   * This acknowledgement system is used to support Secure Digital cards.
   *
   *  - SPI_WAIT_FOR_ACK_BYTE An acknowledgement is received when a received
   *    byte matches @ref acknowledgement_byte.
   *  - SPI_WAIT_FOR_DIFFERNT_ACK An acknowledgement is received when a
   *    received byte differes from @ref acknowledgement_byte.
   *
   * @note This value will be reset tp SPI_NO_ACKNOWLEDGEMNT when @ref
   *       spi_communicate is called.
   */
  uint8_t acknowledgement_mode;

  /**
   * Slave Acknowledgement Indicator.
   * If @ref acknowledgement_mode is not @ref SPI_NO_ACKNOWLEDGEMENT, this
   * byte is used to determine when the slave acknowledges the transfer.
   * The byte used to acknowledge a transfer is stored here when that
   * transfer completes.
   */
  uint8_t acknowledgement_byte;

  /**
   * Slave Acknowledgement Request Byte.
   * When the SPI system is asking the slave for an acknowledgement, it will
   * continually transmit the value indicated in this byte.
   */
  uint8_t acknowledgement_request_byte;

  /**
   * Acknowledgement Try Count
   * Number of times to transmit @ref acknowledgement_byte in an attempt
   * to satisfy the acknowledgement mode. If the slave fails to meet the
   * criteria for the selected acknowledgement mode, the transfer completes
   * anyway. In this case, @ref acknowledgement_received is set to false.
   */
  uint16_t acknowledgement_try_count;

  /**
   * Indicates whether an acknowledgement was received successfully.
   * If @ref acknowledgement_mode is set to something other than
   * SPI_NO_ACKNOWLEDGEMENT, this bit will be set to true when
   * spi_communicate completes this transfer if and only if the slave
   * met the criteria for the selected acknowledgement mode.
   */
  bool acknowledgement_received;

  /**
   * Determines whether to deselect the slave at the end of the transfer.
   * If true, the slave will be deselected (i.e. SS will go high) after
   * @ref transfer_buffer has been transmitted. If
   * @ref wait_for_acknowledgement is true, the SPI system will wait for
   * an acknowledgement, as described in the documentation for that struct
   * member, before deselecting the slave.
   */
  bool deselect_slave_when_finished;

  /**
   * Function to call when the transfer completes
   * If non-NULL, this function will be called when the SPI transfer
   * completes.
   */
  spi_callback_fn transfer_complete_callback;

  /**
   * User-defined storage for use in the complete callback.
   */
  void* transfer_complete_arg;
} SpiTransfer;

/**
 * Initialize a pure SPI port for data transfer.
 * This function sets up an SPI port for data transfer. The baud rate will be
 * selected to be below @c max_clock_speed_hz, or the function will return false
 * if this is impossible. If @c lsb_first is true, data is transmitted LSB
 * first; otherwise it is transmitted MSB first. The SPI bus will behave
 * according to @c spi_mode in terms of data sample and setup and clock
 * polarity.
 *
 * After calling this function, the SS line for the given bus will be held high.
 *
 * @param port The SPI port to initialize.
 * @param lsb_first Specifies the bit order for transmitting data.
 * @param spi_mode Specifies clock polarity and data sample and setup timing.
 * @param max_clock_speed_hz Maximum clock speed, in Hz, to run the SPI bus.
 * @param out_control Control struct to write with setup information
 *
 * @return true if the SPI port was successfully initialized.
 */
bool spi_init(SPI_t* port,
              bool lsb_first,
              uint8_t spi_mode,
              uint32_t max_clock_speed_hz,
              SpiControl* out_control);

/**
 * Perform communication over the SPI bus.
 * This function performs the given transfer operation over the SPI bus
 * according to the values defined in the given SpiTransfer structure.
 *
 * This function will return after the transfer is started. Data is
 * transferred via a low-level interrupt, so the transfer may not be complete
 * when this function returns. Either rely on the callback function or calling
 * spi_is_idle to determine when the transfer completes.
 *
 * @param control SPI control structure associated with the port over which to
 *                communicate.
 * @param transfer_request Transfer request to make.
 * @return true if the
 */
bool spi_communicate(SpiControl* port, SpiTransfer* transfer_request);

/**
 * Check if an SPI port is idle.
 *
 * @param port SPI port to check.
 * @return true if the given SPI port is idle.
 */
bool spi_is_idle(SpiControl* port);

/**
 * Set SPI port to high-impedance mode.
 * No writing can be done while in this mode, but if the SPI pins are directly
 * exposed, calling this function when they are unused can help prevent shorts.
 *
 * Once this function is called, call @ref spi_init before using any other SPI
 * functions on a port, except for those which just read registers and memory,
 * like @ref spi_is_slave_selected and
 */
void spi_set_high_impedance(SPI_t* spi);

/**
 * Check whether the slave select line is low for a given port.
 *
 * @return true if the slave select line for the given SPI port is low.
 */
bool spi_is_slave_selected(SpiSlave* slave);

/**
 * Set the slave select for a given SPI port into output mode.
 */
void spi_set_slave_select_output(SpiSlave* spi, bool output);

/**
 * Select or deselect an SPI slave.
 *
 * @param channel index of the SPI port to address
 * @param selected true to select the slave (SS goes low), false to deselect.
 */
static inline void spi_select_slave(SpiSlave* slave, bool selected) {
  gpio_set(slave->ss, selected ^ !slave->ss_active_high);
}

/**
 * Read the MISO line and report its value.
 *
 * @return true if the MISO line is high, false if the MISO line is low.
 */
bool spi_read_miso(SPI_t* spi);

/**
 * Terminate an ongoing SPI transfer.
 * The SPI transfer in progress will be terminated and the slave
 * deselected.
 *
 * @param port SPI port containing the active transfer to terminate.
 */
void spi_terminate_current_transfer(SpiControl* port);

/**
 * SPI Transfer Interrupt Handler. Handles all logic related to ongoing SPI
 * transfers initiated through this interface. Calls the transfer complete
 * handler when it completes.
 *
 * @param Pointer to the memory location containing the active transfer.
 */
bool spi_interrupt_handler(SpiTransfer** transfer_ptr,
                           volatile uint8_t* transfer_buffer);

#endif // _SPI_H_
