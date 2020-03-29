/*
 * Copyright (c) 2020, NXP. All rights reserved.
 * Distributed under The MIT License.
 * Author: Abraham Rodriguez <abraham.rodriguez@nxp.com>
 */

/**
 * @file
 * Build configurations header file for setting up the driver in
 * function of which MCU from the S32K14x familiy is being used for,
 * which differ in the number of CAN-FD capable FlexCAN instances as shown:
 *
 * S32K142: 1
 * S32K144: 1
 * S32K146: 2
 * S32K148: 3
 */

#ifndef S32K_BUILD_CONFIG_HPP_INCLUDED
#define S32K_BUILD_CONFIG_HPP_INCLUDED

/**
 * Macro for additional configuration needed when using a TJA1044 transceiver, which is used
 * in NXP's UCANS32K146 board, set to 0 when using EVB's or other boards.
 */
#ifndef UAVCAN_NODE_BOARD_USED
#    define UAVCAN_NODE_BOARD_USED 1
#endif

/**
 * Include desired target S32K14x memory map header file dependency,
 * defaults to S32K146 from NXP's UCANS32K146 board
 */
#include "S32K146.h"

#endif  // S32K_BUILD_CONFIG_HPP_INCLUDED