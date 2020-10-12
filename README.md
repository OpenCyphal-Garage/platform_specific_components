# Platform-specific components

[![Build Status](https://travis-ci.org/UAVCAN/platform_specific_components.svg?branch=master)](https://travis-ci.org/UAVCAN/platform_specific_components)
[![Forum](https://img.shields.io/discourse/users.svg?server=https%3A%2F%2Fforum.uavcan.org&color=1700b3)](https://forum.uavcan.org)

This repository contains various platform-specific components maintained by the UAVCAN Development Team.
The quality and the level of support provided for these components may be substantially lower than
that of the UAVCAN implementation libraries.

The content is organized into directories following the pattern `/<platform>/<library>/`;
for example, the Libuavcan driver for STM32 can be found under `/stm32/libuavcan/`.
Further segregation may be defined on a per-directory basis.

All code is MIT-licensed unless a dedicated LICENSE file is provided.

## See also

- [107-Arduino-MCP2515](https://github.com/107-systems/107-Arduino-MCP2515) -- Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
- Feel free to add more references.

Users are encouraged to search through this repository for code that can be used with their target platform
and use it as a starting point or as a practical guideline in the development of a customized solution for
the specific application.
If nothing is found, consider checking the `legacy-v0` branch: the code contained there may require heavy modification
but it might still be useful.
