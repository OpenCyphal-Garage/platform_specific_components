Libuavcan STM32 platform driver
===============================

[![Gitter](https://img.shields.io/badge/gitter-join%20chat-green.svg)](https://gitter.im/UAVCAN/general)

The directory `driver` contains the [STM32](http://www.st.com/stm32) platform driver for
[Libuavcan](http://uavcan.org/Implementations/Libuavcan).

The libuavcan driver for STM32 is a C++ library with no third-party dependencies that provides bare-metal
drivers for the following:

* Built-in CAN interfaces (singly or doubly redundant, depending on the MCU model).
* One of the general-purpose timers (TIM2...TIM7); the driver also supports clock synchronization.

The following RTOS are supported:

* [ChibiOS](http://chibios.org/)
* [NuttX](http://nuttx.org/)
* [FreeRTOS](http://freertos.org/)
* Bare metal (no OS)

This driver should work with any STM32 series MCU. The following models were tested so far:

* STM32F103
* STM32F105
* STM32F107
* STM32F205
* STM32F303
* STM32F427
* STM32F446

Virtually any standard-compliant C++ compiler for ARM Cortex-Mx can be used.
The library was mainly tested with GCC 4.7 and newer so far, but any other compiler should work too.

This driver can be used as a good starting point if a custom driver needs to be implemented for a
particular application.

## Building

No extra dependencies are required. The libuavcan library need not be installed on the host system.

### Using Make

The general approach is as follows:

1. Include the libuavcan core makefile (`include.mk`) to obtain the following:
  1. List of C++ source files - make variable `$(LIBUAVCAN_SRC)`
  2. C++ include directory path - make variable `$(LIBUAVCAN_INC)`
  3. DSDL compiler executable path (which is a Python script) - make variable `$(LIBUAVCAN_DSDLC)`
2. Include the libuavcan STM32 driver makefile (`include.mk`) to obtain the following:
  1. List of C++ source files - make variable `$(LIBUAVCAN_STM32_SRC)`
  2. C++ include directory path - make variable `$(LIBUAVCAN_STM32_INC)`
3. Invoke the DSDL compiler using the variable `$(LIBUAVCAN_DSDLC)`.
4. Add the output directory of the DSDL compiler to the list of include directories.
5. Add the obtained list of C++ source files and include directories to the application sources or
build an independent static library.

The relevant part of the makefile implementing the steps above is provided below.
This example assumes that the UAVCAN repository is available in the same directory where the application's
makefile resides (e.g., as a git submodule).

```make
#
# Application
#

CPPSRC = src/main.cpp   # Application source files

UINCDIR = include       # Application include directories

UDEFS = -DNDEBUG        # Application preprocessor definitions

#
# UAVCAN library
#

UDEFS += -DUAVCAN_STM32_CHIBIOS=1      \ # Assuming ChibiOS for this example
         -DUAVCAN_STM32_TIMER_NUMBER=6 \ # Any suitable timer number
         -DUAVCAN_STM32_NUM_IFACES=2     # Number of CAN interfaces to use (1 - use only CAN1; 2 - both CAN1 and CAN2)

# Add libuavcan core sources and includes
include uavcan/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

# Add STM32 driver sources and includes
include libuavcan_stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_STM32_INC)

# Invoke DSDL compiler and add its default output directory to the include search path
$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
UINCDIR += dsdlc_generated      # This is where the generated headers are stored by default

#
# ...makefile continues
#
```

### Using other build systems

One possible approach is the following:

* Manually add the libuavcan core include directory to the project includes.
The path is `uavcan/libuavcan/include`.
* Manually add the STM32 driver include directory to the project includes.
The path is `libuavcan_stm32/driver/include`.
* Manually invoke the DSDL compiler and add the output directory to the project includes.
Alternatively, configure the build system to invoke the compiler automatically.
* Add all the C++ source files of libuavcan core and STM32 driver to the project.

## Build configuration

The driver is configurable via preprocessor definitions.
Some definitions must be provided in order for the build to succeed;
some others have default values that can be overridden only if needed.

The most important configuration options are documented in the table below.

Name                            | RTOS              | Description
--------------------------------|-------------------|--------------------------------------------------------------------------
`UAVCAN_STM32_CHIBIOS`          | ChibiOS           | Set to 1 if using ChibiOS.
`UAVCAN_STM32_NUTTX`            | NuttX             | Set to 1 if using NuttX.
`UAVCAN_STM32_FREERTOS`         | FreeRTOS          | Set to 1 if using FreeRTOS.
`UAVCAN_STM32_BAREMETAL`        | N/A               | Set to 1 if not using any OS.
`UAVCAN_STM32_IRQ_PRIORITY_MASK`| ChibiOS, FreeRTOS | This option defines IRQ priorities for the CAN controllers and the timer. Default priority level is one lower than the kernel priority level (i.e. max allowed level).
`UAVCAN_STM32_TIMER_NUMBER`     | Any               | Hardware timer number reserved for clock functions. If this value is not set, the timer driver will not be compiled; this allows the application to implement a custom driver if needed. Valid timer numbers are 2, 3, 4, 5, 6, and 7.
`UAVCAN_STM32_NUM_IFACES`       | Any               | Number of CAN interfaces to use. Valid values are 1 and 2.

## Bare metal driver

If you are not using any OS, you must supply your own `chip.h` header file to configure the STM32 driver for your particular MCU.

A `chip.h` file includes the HAL header file for your MCU (e.g. `#include "stm32f3xx.h"`) and sets the preprocessor definitions in the table below. Some of the preprocessor definitions may already be defined in the HAL header files for your MCU or by libuavcan.

Name                   | Description
-----------------------|-------------------------------------------------------------------------------------------------------
`STM32?XX`             | The STM32 family of the MCU (e.g. `#define STM32F2XX` for F2 family MCUs).
`STM32_PCLK1`          | PCLK1 frequency in Hertz. Calling `HAL_RCC_GetPCLK1Freq()` returns this value.
`STM32_TIMCLK1`        | TIMCLK1 frequency in Hertz.
`CAN1_TX_IRQHandler`, `CAN1_RX0_IRQHandler`, `CAN1_RX1_IRQHandler` | Name of the TX, RX0 and RX1 interrupt handlers for the first CAN interface.
`CAN2_TX_IRQHandler`, `CAN2_RX0_IRQHandler`, `CAN2_RX1_IRQHandler` | Name of the TX, RX0 and RX1 interrupt handlers for the second CAN interface. Only applicable if `UAVCAN_STM32_NUM_IFACES` is 2.

### Example

This example is for an STM32F446 MCU.

```
#include "stm32f4xx.h"  // HAL header file

#define STM32F4XX
#define STM32_PCLK1           (42000000ul)          // 42 MHz
#define STM32_TIMCLK1         (84000000ul)          // 84 MHz
#define CAN1_TX_IRQHandler    CAN1_TX_IRQHandler
#define CAN1_RX0_IRQHandler   CAN1_RX0_IRQHandler
#define CAN1_RX1_IRQHandler   CAN1_RX1_IRQHandler
```

## Usage examples

The following firmware projects can be used as a reference:

* [Sapog](https://github.com/PX4/sapog)
* [Zubax GNSS](https://github.com/Zubax/zubax_gnss)
* [PX4](https://github.com/PX4/Firmware)
