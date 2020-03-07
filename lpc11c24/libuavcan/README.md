Libuavcan v0 platform driver for NXP LPC11C24
=============================================

This document describes the Libuavcan v0 driver for the
[NXP LPC11C24](http://www.nxp.com/products/microcontrollers/cortex_m0_m0/lpc1100/LPC11C24FBD48.html)
MCU and shows how to use Libuavcan v0 on this platform.

NXP LPC11C24 is a low-end ARM Cortex-M0 microcontroller that features an embedded CAN controller with a CAN transceiver,
rendering it a true single-chip solution for CAN applications.
Since the computational resources of this MCU are quite limited, a number of restrictions apply:

* The driver is designed to run on bare metal - no RTOS supported.
* The libuavcan library can be compiled only in tiny mode, which removes a number of auxiliary features.
* Core clock frequency must be strictly 48 MHz.
* The driver supports time synchronization in slave mode only, i.e.,
the device running libuavcan with this driver can't act as a clock synchronization master.

The following hardware modules are used by the driver:

* CAN controller
* SysTick timer for clock functions

The driver is written in standard C++11.
It leverages the [LPCOpen](http://www.lpcware.com/lpcopen) libraries for low-level hardware control.

Users are encouraged to use the test project for this driver
as a starting point in developing their own applications (see the next sections).

## Build configuration

This driver can be configured via the preprocessor definitions listed in the table below.
All of these options have adequate default settings, so in most cases they need not be altered.

Name                            | Default | Description
--------------------------------|---------|----------------------------------------------------------------------
`UAVCAN_LPC11C24_RX_QUEUE_LEN`  | 8       | CAN driver RX buffer depth
`UAVCAN_LPC11C24_USE_WFE`       |         | This option enables execution of the WFE instruction when the driver is blocked in `select()`. This feature is disabled by default. Please read the relevant section below.

### `UAVCAN_LPC11C24_USE_WFE`

Generally, this feature should be avoided in real-time applications, so it's disabled by default.

Pros:

* Lower power consumption, because the core will halt on WFE if there are no pending tasks instead of burning cycles in a busy loop.

Cons:

* It is hard to control the duration of the WFE block, which may disrupt hard real-time processing.
* Sometimes, WFE may extend the `select()` blocking timeout past the requested duration.

## Example

Please use the test application distributed with this driver as a starting point in developing your own
firmware for this platform. It can be easily adapted for any hardware platform.

To build the example using vagrant do:

```
vagrant up
vagrant ssh
cd test_olimex_lpc_p11c24
make
```
