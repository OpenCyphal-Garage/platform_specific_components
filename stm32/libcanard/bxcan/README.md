# Libcanard v1 driver for STM32

This is a compact and simple CAN driver for STM32 microcontrollers that embed the bxCAN macrocell.
It has no dependencies besides a tiny part of the standard C library,
and useable with virtually any operating system or on bare metal.

This driver should be compatible with all STM32 microcontrollers that leverage bxCAN, but not FDCAN.
Please consult with the applicable documentation from ST Microelectronics for details.
So far this driver has been tested at least with the following MCU:

* STM32L431
* STM32F446
* Please extend this list if you used it with other MCU.

## Features

* Proper handling of the TX queue prevents inner priority inversion.
* Dependency free, works with any OS and on bare metal.
* Compact, suitable for ROM and RAM limited applications (e.g. bootloaders).
* Does not use IRQ and critical sections at all.
* Non-blocking API.
* Supports both CAN1 and CAN2 simultaneously.
* Supports hardware acceptance filters.
* Supports proper CAN bus timing configuration in a user friendly way.

## Caveats

Some design trade-offs have been made in order to provide the above features.

* The RX FIFO is only 3 CAN frames deep, since this is the depth provided by the CAN hardware.
In order to avoid frame loss due to RX overrun, the following measures should be adopted:
  * Use hardware acceptance filters - the driver provides a convenient API to configure them.
  * Read the queue at least every 3x minimum frame transmission intervals.
    For a 1 Mbps bus, the polling interval should not exceed
    [192 microseconds](https://github.com/Zubax/kocherga/issues/5).
    If an RTOS is used, the tick interval may need to be set appropriately.
* The driver does not permit concurrent access from different threads of execution.
* The clocks of the CAN peripheral must be enabled by the application before the driver is
initialized. The driver cannot do that because this logic is not uniform across the STM32 family.

Note that it is possible to invoke the driver's API functions from IRQ context, provided that the
application takes care about proper guarding with critical sections.
This approach allows the application to read the RX queue from an external CAN IRQ handler.

## How to use

The driver is so simple its entire documentation is defined in the header file.
Please do endeavor to read it.

[Use code search to find real life usage examples using the keyword `bxCANConfigure`](https://github.com/search?q=bxCANConfigure&type=Code).
