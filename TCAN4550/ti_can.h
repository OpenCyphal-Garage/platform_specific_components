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

///////////////////////////////////////////////////////////////////////////////////////////

/// 0 - CAN, 1 - CAN FD
#define CAN_MODE 1

/// 0 - Bit Rate Switching disabled, 1 - enabled
#define BRS 1
#define CCCR_BRSE (1U << 9U)
 
/// 0 - Flexible Datarate disabled, 1 - enabled
#define FD 1
#define CCCR_FDOE (1U << 8U)

///////////////////////////////////////////////////////////////////////////////////////////

/// BIT TIMING VALUES

// FOR CAN
#define NBTP 0x101C

#define CAN_NBTP_NSJW_SHFT (25U)
#define CAN_NBTP_NSJW_MASK (0x7FU << CAN_NBTP_NSJW_SHFT)
#define CAN_SYNC_JUMP_WIDTH(x) ((uint8_t)(x) << CAN_NBTP_NSJW_SHFT)

#define CAN_NBTP_NTSEG1_SHFT (8U)
#define CAN_NBTP_NTSEG1_MASK (0xFFU << CAN_NBTP_NTSEG1_SHFT)
#define CAN_TIME_SEG_1(x) ((uint8_t)(x) << CAN_NBTP_NTSEG1_SHFT)

#define CAN_NBTP_NTSEG2_SHFT (0U)
#define CAN_NBTP_NTSEG2_MASK (0x7FU << CAN_NBTP_NTSEG2_SHFT)
#define CAN_TIME_SEG_2(x) ((uint8_t)(x) << CAN_NBTP_NTSEG2_SHFT)

#define CAN_NBTP_NBRP_SHFT (0U)
#define CAN_NBTP_NBRP_MASK (0x7FU << CAN_NBTP_NBRP_SHFT)
#define CAN_PRESCALER(x) ((uint8_t)(x) << CAN_NBTP_NBRP_SHFT)

// FOR CAN-FD
#define DBTP 0x100C 
#define TDCR 0x1048 // Transmission Delay Compensation Register

#define CANFD_DBTP_DSJW_SHFT (0U)
#define CANFD_DBTP_DSJW_MASK (0xFU << CANFD_DBTP_DSJW_SHFT)
#define CANFD_SYNC_JUMP_WIDTH(x) ((uint8_t)(x) << CANFD_DBTP_DSJW_SHFT)

#define CANFD_DBTP_DTSEG1_SHFT (8U)
#define CANFD_DBTP_DTSEG1_MASK (0x1FU << CANFD_DBTP_DTSEG1_SHFT)
#define CANFD_TIME_SEG_1_WIDTH(x) ((uint8_t)(x) << CANFD_DBTP_DTSEG1_SHFT)

#define CANFD_DBTP_DTSEG2_SHFT (4U)
#define CANFD_DBTP_DTSEG2_MASK (0xFU << CANFD_DBTP_DTSEG2_SHFT)
#define CANFD_TIME_SEG_2_WIDTH(x) ((uint8_t)(x) << CANFD_DBTP_DTSEG2_SHFT)

#define CANFD_DBTP_DBRP_SHFT (16U)
#define CANFD_DBTP_DBRP_MASK (0xFU << CANFD_DBTP_DBRP_SHFT)
#define CANFD_PRESCALER(x) ((uint8_t)(x) << CANFD_DBTP_DBRP_SHFT)

#define CANFD_TDCR_TDCO_SHFT (8U)
#define CANFD_TDCR_TDCO_MASK (0x7FU << CANFD_TDCR_TDCO_SHFT)
#define CANFD_DELAY_COMPENSATION_OFFSET(x) ((uint8_t)(x) << CANFD_TDCR_TDCO_SHFT)

// Bit Timing Parameters
typedef struct
{
    uint8_t prescaler;
    uint8_t prop_and_phase1;
    uint8_t phase2;
    uint8_t sync_jump_width;
    uint8_t tdc;
} BitTimingParams;

///////////////////////////////////////////////////////////////////////////////////////////

/// MESSAGE RAM

// MRAM Registers' Addresses
#define SIDFC 0x1084 // 11-bit Filter (SID Filter) 
#define XIDFC 0x1088 // 29-bit Filter (XID Filter)
#define RXF0C 0x10A0 // Rx FIFO 0
#define RXF1C 0x10B0 // Rx FIFO 1
#define RXBC  0x10AC // Rx Buffers
#define RXESC 0x10BC // Rx Element Size Config
#define TXEFC 0x10F0 // Tx Event FIFO
#define TXBC  0x10C0 // Tx Buffers
#define TXESC 0x10C8 // Tx Element Size Config

// Bits In Registers

//
// //
//

// Message RAM Design Parameters 
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

// Filters Configuration: SID and XID (Standard and Extended)

// Number of Standard Acceptance Filters
#define CAN_SID_NUM_ACCEPTANCE_FILTERS 14U

// Number of Extended Acceptance Filters
#define CAN_XID_NUM_ACCEPTANCE_FILTERS 14U

// SID
typedef struct
{
    uint32_t sid_id;
    uint32_t sid_mask;
} CAN_SIDFilterParams;

#define CAN_SFID1_SHFT (16U)
#define CAN_SFID1_MASK (0x7FF << CAN_SFID1_SHFT)
#define CAN_SID_FILTER_1(x) ((uint8_t)x << CAN_SFID1_SHFT)

#define CAN_SFID2_SHFT (0U)
#define CAN_SFID2_MASK (0x7FF << CAN_SFID2_SHFT)
#define CAN_SID_FILTER_2(x) ((uint8_t)x << CAN_SFID2_SHFT)


// XID
typedef struct
{
    uint32_t xid_id;
    uint32_t xid_mask;
} CAN_SIDFilterParams;

///////////////////////////////////////////////////////////////////////////////////////////

/// Device Initialization Mode

// Device Mode Selection Register Address
#define MODE_SEL 0x0800

// Available Device Modes
#define STANDBY_MODE (0b01 << 6U)
#define NORMAL_MODE (0b10 << 6U)

// CC Control Register Address
#define CCCR 0x1018 

// Bits Necessary For Initialization
#define CAN_CCCR_INIT (1U << 0U)
#define CAN_CCCR_CCE  (1U << 1U)
#define CAN_CCCR_CSR  (1U << 4U)

///////////////////////////////////////////////////////////////////////////////////////////