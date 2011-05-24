libAVR
======

An open-source, performant, retargettable replacement for Arduino base
libraries. libAVR emphasizes straight-up-C and low overhead while maintaining
extreme modularity and acceptable performance.

Project Status
--------------

I'm a graduating senior, so I don't have much time at the moment. I built this
for a telemetry package based on the XMEGA architecture, but I'm slowly
porting the good parts over to support any AVR architecture. Eventually, I
plan the following support in this library:

 * Generic Utilities
    * Fastest-possible descriptive GPIO (`HIGH(hardcodedPinName)` compiles to 1 instruction)
    * Rich printf-like debugging support via USART
    * Status LED support
    * Industry-standard numeric parsing routines
    * CRC7 and CRC16
    * Endianness convenience routines
    * Precise lat-long data structures
 * Data Structures
    * Fast (interrupt latency-acceptable) circular buffers
    * Linked-lists
    * Heaps
 * Rich Peripheral Support
    * Onboard ADC
    * SPI
        * Linux-like driver for rich interrupt-driven tasks (see below)
        * Extended polling via interrupts
    * Hardware Timers
        * Signal Decoding
    * TWI
    * USART
        * "Asynchronous" communication via interrupts
        * Buffer watching (think: How many newlines have been received?)
        * Standard synchronous communication
       * RTS/CTS signalling for transmission
       * VT100 support
 * Offboard Sensor support
    * ADS1147 ADC
    * BMP085
    * LIS302/LIS331 3-axis accelerometers
    * HMC5883 3-axis magnetometer
    * IMU3000 (basic support, no DMP)
 * Secure Digital and microSD support
    * Interrupt-driven!
 * GPS NMEA parsing
    * Lat/Long, altitude, 3-d velocity
    * From USART or arbitrary buffer

