# Platform-specific components

[![Main Workflow](https://github.com/OpenCyphal-Garage/platform_specific_components/actions/workflows/main.yml/badge.svg)](https://github.com/OpenCyphal-Garage/platform_specific_components/actions/workflows/main.yml)
[![Forum](https://img.shields.io/discourse/users.svg?server=https%3A%2F%2Fforum.opencyphal.org&color=1700b3)](https://forum.opencyphal.org)

This repository contains various platform-specific components maintained by the OpenCyphal team.
The quality and the level of support provided for these components may be substantially lower than
that of the official OpenCyphal implementation libraries.

The content is organized into directories following the pattern `/<platform>/<library>/`;
for example, the Libuavcan driver for STM32 can be found under `/stm32/libcanard/`.
Further segregation may be defined on a per-directory basis.

All code is MIT-licensed unless a dedicated LICENSE file is provided.

## See also

- [107-Arduino-MCP2515](https://github.com/107-systems/107-Arduino-MCP2515) -- Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
- Feel free to add more references.

Users are encouraged to search through this repository for code that can be used with their target platform
and use it as a starting point or as a practical guideline in the development of a customized solution for
the specific application.
