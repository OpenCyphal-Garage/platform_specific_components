# Platform-specific components

[![Forum](https://img.shields.io/discourse/users.svg?server=https%3A%2F%2Fforum.uavcan.org&color=1700b3)](https://forum.uavcan.org)

This repository contains various platform-specific components maintained by the UAVCAN Development Team.
The quality and the level of support provided for these components may be substantially lower than
that of the UAVCAN implementation libraries.

The content is organized into directories following the pattern `/<platform>/<library>/`;
for example, the Libuavcan driver for STM32 can be found under `/stm32/libuavcan/`.
Further segregation may be defined on a per-directory basis.

Users are encouraged to search this repository for the code that can be used with their target platform
and use it as a starting point or as a practical guideline in the development of a customized solution for
the specific application.
If nothing is found, consider checking the legacy-v0 branch: the code contained there may require heavy modification
but it might still be useful.

All code is MIT-licensed unless a dedicated LICENSE file is provided.
