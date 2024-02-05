///                            ____                   ______            __          __
///                           / __ `____  ___  ____  / ____/_  ______  / /_  ____  / /
///                          / / / / __ `/ _ `/ __ `/ /   / / / / __ `/ __ `/ __ `/ /
///                         / /_/ / /_/ /  __/ / / / /___/ /_/ / /_/ / / / / /_/ / /
///                         `____/ .___/`___/_/ /_/`____/`__, / .___/_/ /_/`__,_/_/
///                             /_/                     /____/_/
///
/// A low-level driver with the main purpose of helping new hardware revisions or new designs adopt CAN FD
/// without having to port an existing codebase to a new platform.
///
/// Author: Daniil Bragin
///


#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/// 0 - CAN, 1 - CAN FD
#define CAN_MODE 1 

/// Bit Timing Registers (Addresses)
#define NBTP 0x101C // For CAN
#define DBTP 0x100C // For CAN FD
#define TDCR 0x1048 // Transmissio Delay Compensation Register (for CAN FD only)




/// CAN Bit Timing Values
typedef struct
{
    uint8_t prescaler;
    uint8_t prop_and_phase1;
    uint8_t phase2;
    uint8_t sync_jump_width;
} BitTimingParams;


/// Message RAM Design Parameters 
typedef struct
{
    uint8_t sid_filters;     /// 11-bit filter size
    uint8_t xid_filters;     /// 29-bit filter size
    uint8_t rx_fifo_0;       
    uint8_t rx_fifo_1;
    uint8_t rx_buffers;
    uint8_t tx_event_fifo;
    uint8_t tx_buffers;
} TiMRAMParams;


