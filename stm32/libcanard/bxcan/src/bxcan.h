///                         __   __   _______   __   __   _______   _______   __   __
///                        |  | |  | /   _   ` |  | |  | /   ____| /   _   ` |  ` |  |
///                        |  | |  | |  |_|  | |  | |  | |  |      |  |_|  | |   `|  |
///                        |  |_|  | |   _   | `  `_/  / |  |____  |   _   | |  |`   |
///                        `_______/ |__| |__|  `_____/  `_______| |__| |__| |__| `__|
///                            |      |            |         |      |         |
///                        ----o------o------------o---------o------o---------o-------
///
/// A generic low-level driver for the STM32 bxCAN macrocell. Designed to support any operating system and baremetal
/// systems. This driver is not compatible with FDCAN controllers available in newer STM32 MCUs such as STM32H7.
/// The driver has no external dependencies besides a tiny subset of the C standard library.
///
/// This driver exclusively supports extended-id frame format (29-bit) data frames. There is no support for the
/// classic-id (11-bit) data frames, remote frames, overload frames, or error frames. The 29 bits of the extended
/// identifier values used in this driver match the order of the CAN protocol, as described in the CAN 2.0B standard.
///
/// The driver does not use interrupts or critical sections internally, which makes it a good fit for certain hard
/// real-time systems where interrupts may be impossible to leverage (e.g., some motor control applications).
/// The application shall poll the RX queue with a sufficient frequency to prevent RX overrun. Given that the hardware
/// RX FIFO is three frames deep, the polling interval shall not exceed (shortest frame transmission time) x3.
/// At 1 Mbit/s for CAN 2.0B, this amounts to 192 microseconds. The application may be able to slightly extend
/// the interval by configuring the acceptance filters appropriately.
///
/// If the application is unable to poll the driver with a sufficient frequency, it is possible to enable the
/// interrupts and invoke the driver from there. It is safe since the driver is context-agnostic. Same holds for
/// the transmission management: it can be done by polling or from an interrupt depending on the timing requirements.
/// If interrupts are used, it is the responsibility of the caller to ensure adequate synchronization.
///
/// The driver automatically aborts all pending transmissions if the CAN controller enters the bus-off state.
/// This behavior is introduced to support the plug-and-play node-ID allocation protocol defined by UAVCAN.
///
/// The driver does not support TX timestamping. This feature is only required for time synchronization masters.
/// The suggested alternative is to implement the required logic at the driver layer directly, as is often done
/// with some other precise time protocols such as IEEE 1588.
///
/// The clocks of the CAN peripheral shall be enabled by the application before the driver is initialized.
/// The driver cannot do that because this logic is not uniform across the STM32 family.
///
/// The CAN interface index counts from zero, as per the C convention. ST numbers the CAN peripherals starting
/// from 1. Thus: iface_index 0 = bxCAN1 and iface_index 1 = bxCAN2.
///
/// The recommended approach for integration is to copy-paste the source files into the project tree and modify
/// as necessary. The .c file does not require any specific build options.
///
/// This software is distributed under the terms of the MIT License. Copyright (c) 2020 UAVCAN Development Team.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// This is defined by the bxCAN hardware.
/// Devices with only one CAN interface have 14 filters (e.g. F103).
/// Devices with two CAN interfaces (e.g. F105, F446) have 28 filters, which are shared equally.
#define BXCAN_NUM_ACCEPTANCE_FILTERS 14U

/// A CAN acceptance filter configuration. Usable only with extended CAN 2.0B data frames.
/// It is not possible to configure acceptance of standard-ID or RTR frames.
/// The bits above 29-th shall be zero. Special case: if both mask and ID are zero, the filter will reject all frames.
typedef struct
{
    uint32_t extended_id;
    uint32_t extended_mask;
} BxCANFilterParams;

/// Bit timing parameters. Use bxCANComputeTimings() to derive these from the desired bus data rate.
/// Some applications may prefer to store pre-computed parameters instead of calculating them at runtime.
typedef struct
{
    uint16_t bit_rate_prescaler;     /// [1, 1024]
    uint8_t  bit_segment_1;          /// [1, 16]
    uint8_t  bit_segment_2;          /// [1, 8]
    uint8_t  max_resync_jump_width;  /// [1, 4] (recommended value is 1)
} BxCANTimings;

/// Initialization can be performed multiple times to switch between operating modes and/or bit rates.
/// For example, during the automatic bit-rate detection phase, the application may cycle through several
/// pre-configured bit rates (typically 1000/500/250/125 kbps) in silent mode until a match is found.
/// Returns true on success, false if initialization failed (INAK timeout).
///
/// The initial state of the acceptance filters is to accept everything.
/// Use bxCANConfigureFilters() to override this after the interface is configured.
///
/// WARNING: The clock of the CAN module must be enabled before this function is invoked!
///          If CAN2 is used, CAN1 must be also enabled!
///
/// WARNING: The driver is not thread-safe!
///          It does not use IRQ or critical sections though, so it is safe to invoke its API functions from the
///          IRQ context from the application.
bool bxCANConfigure(const uint8_t iface_index, const BxCANTimings timings, const bool silent);

/// Acceptance filter configuration. Unused filters shall be set to {0, 0} (all bits zero); they will reject all frames.
/// When the interface is reinitialized, hardware acceptance filters are reset, so this function shall be re-invoked.
/// While reconfiguration is in progress, some received frames may be lost.
/// Filters alternate between FIFO0/1 in order to equalize the load: even filters take FIFO0, odd filters take FIFO1.
/// This will cause occasional priority inversion and frame reordering on reception, but that is acceptable for UAVCAN,
/// and most other CAN-based protocols will tolerate this too since there will be no reordering within the same CAN ID.
void bxCANConfigureFilters(const uint8_t iface_index, const BxCANFilterParams params[BXCAN_NUM_ACCEPTANCE_FILTERS]);

/// This function is intended for error statistics tracking. The read is destructive.
/// Returns true if at least one of the following events took place since the previous invocation:
///     - TX frame aborted on timeout (deadline expiration).
///     - RX FIFO overrun.
///     - Bus error reported by the CAN controller.
bool bxCANReapError(const uint8_t iface_index);

/// Schedule a CAN frame for transmission.
///
/// The function takes the current time and the time when the scheduled frame will have to be aborted.
/// The time units may be arbitrary as long as it is guaranteed that the values will never overflow (typ. microseconds).
/// When looking for a free transmission slot, the function will ensure that none of the currently pending TX mailboxes
/// contain frames that have timed out (i.e., have not been transmitted before the deadline). If such mailboxes are
/// found they are aborted. This is done BEFORE the new frame is scheduled in order to free up expired mailboxes for
/// the new frame.
///
/// The return value is true if the frame is accepted for transmission, false otherwise. Acceptance logic:
///     - If all mailboxes are free, the frame is accepted.
///     - If all mailboxes are pending (busy), the frame is rejected.
///     - If at least one mailbox is pending, the frame is accepted only if its CAN ID value is smaller than that of
///       all pending mailboxes. This is done to avoid priority inversion; see the UAVCAN Specification for details.
///     - If current_time > deadline, the frame is discarded, TX timeout error is registered, and true is returned.
///
/// This interface does not support the transmission of CAN 2.0A (standard-ID) frames.
///
/// Redundantly interfaced configurations shall maintain a backlog queue to account for the fact that different
/// interfaces may be unable to transmit frames simultaneously due to different bus load (in terms of frames per
/// second). It is assumed that the outgoing frames are kept in a single prioritized transmission queue, which has
/// to be split into two queues to support different bus loads per interface. One approach is to implement naive
/// copying into two queues, but this approach leads to increased memory utilization. A more conservative approach
/// is to keep a unified queue keeping the outgoing frames for both interfaces, and a single backlog queue keeping
/// the frames for the interface that is lagging behind. This way, no memory overutilization will occur. One backlog
/// queue is sufficient since the faster interface will be fed directly from the shared queue. Graphically:
///
///     [FRAME SOURCE] ---> [ SHARED QUEUE ] --------------------------> [FAST INTERFACE]
///                                           |
///                                            ---> [BACKLOG QUEUE] ---> [SLOW INTERFACE]
///
/// Where "FRAME SOURCE" is typically a UAVCAN/CAN implementation library (like Libcanard), and "SHARED QUEUE" is the
/// main prioritized transmission queue (like the one maintained by Libcanard). Which interface is the FAST one and
/// which one is the SLOW one cannot be known in advance, obviously. They may change roles depending on the bus load
/// conditions. The backlog queue may be implemented as a naive static array that is scanned whenever a frame is
/// inserted or extracted; the linear time complexity may be acceptable because typically the backlog queue is small.
///
/// One should note that the need for maintaining a separate backlog queue arises out of the limitations of the bxCAN
/// macrocell. Certain advanced CAN controllers are equipped with a sufficiently deep hardware transmission queue
/// that relieves the application from manual queue management.
bool bxCANPush(const uint8_t     iface_index,
               const uint64_t    current_time,
               const uint64_t    deadline,
               const uint32_t    extended_can_id,
               const size_t      payload_size,
               const void* const payload);

/// Extract one frame from the RX FIFOs. FIFO0 checked first.
/// The out_payload memory shall be large enough to accommodate the largest CAN frame payload.
/// Returns true if received; false if both RX FIFOs are empty.
bool bxCANPop(const uint8_t   iface_index,
              uint32_t* const out_extended_can_id,
              size_t* const   out_payload_size,
              void* const     out_payload);

/// Given the bxCAN macrocell clock rate and the desired bit rate, compute the optimal timing register configuration.
/// The solution is optimized per the recommendations given in the specifications of DS-015, DeviceNet, CANOpen.
/// Units are SI. Typically, CAN is clocked from PCLK1.
/// Returns false if the requested bit rate cannot be set up at the current clock rate.
/// Returns true on success.
bool bxCANComputeTimings(const uint32_t      peripheral_clock_rate,
                         const uint32_t      target_bitrate,
                         BxCANTimings* const out_timings);

#ifdef __cplusplus
}
#endif
