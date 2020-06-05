///                         __   __   _______   __   __   _______   _______   __   __
///                        |  | |  | /   _   ` |  | |  | /   ____| /   _   ` |  ` |  |
///                        |  | |  | |  |_|  | |  | |  | |  |      |  |_|  | |   `|  |
///                        |  |_|  | |   _   | `  `_/  / |  |____  |   _   | |  |`   |
///                        `_______/ |__| |__|  `_____/  `_______| |__| |__| |__| `__|
///                            |      |            |         |      |         |
///                        ----o------o------------o---------o------o---------o-------
///
/// This is a basic adapter library that bridges Libcanard with SocketCAN.
/// Read the API documentation for usage information.
///
/// To integrate the library into your application, just copy-paste the c/h files into your project tree.
///
/// --------------------------------------------------------------------------------------------------------------------
///
/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2020 UAVCAN Development Team.
/// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#ifndef SOCKETCAN_H_INCLUDED
#define SOCKETCAN_H_INCLUDED

#include "canard.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/// File descriptor alias.
typedef int SocketCANFD;

/// Initialize a new non-blocking (sic!) SocketCAN socket and return its handle on success.
/// On failure, a negated errno is returned.
/// To discard the socket just call close() on it; no additional de-initialization activities are required.
/// The argument can_fd enables support for CAN FD frames.
SocketCANFD socketcanOpen(const char* const iface_name, const bool can_fd);

/// Enqueue a new extended CAN data frame for transmission.
/// Block until the frame is enqueued or until the timeout is expired.
/// Zero timeout makes the operation non-blocking.
/// Returns 1 on success, 0 on timeout, negated errno on error.
int16_t socketcanPush(const SocketCANFD fd, const CanardFrame* const frame, const CanardMicrosecond timeout_usec);

/// Fetch a new extended CAN data frame from the RX queue.
/// If the received frame is not an extended-ID data frame, it will be dropped and the function will return early.
/// The payload pointer of the returned frame will point to the payload_buffer. It can be a stack-allocated array.
/// The payload_buffer_size shall be large enough (64 bytes is enough for CAN FD), otherwise an error is returned.
/// The timestamp of the received frame will be set to the CLOCK_TAI sampled near the moment of its arrival.
/// The function will block until a frame is received or until the timeout is expired. It may return early.
/// Zero timeout makes the operation non-blocking.
/// Returns 1 on success, 0 on timeout, negated errno on error.
int16_t socketcanPop(const SocketCANFD       fd,
                     CanardFrame* const      out_frame,
                     const size_t            payload_buffer_size,
                     void* const             payload_buffer,
                     const CanardMicrosecond timeout_usec);

/// The configuration of a single extended 29-bit data frame acceptance filter.
/// Bits above the 29-th shall be cleared.
typedef struct SocketCANFilterConfig
{
    uint32_t extended_id;
    uint32_t mask;
} SocketCANFilterConfig;

/// Apply the specified acceptance filter configuration.
/// Note that it is only possible to accept extended-format data frames.
/// The default configuration is to accept everything.
/// Returns 0 on success, negated errno on error.
int16_t socketcanFilter(const SocketCANFD fd, const size_t num_configs, const SocketCANFilterConfig* const configs);

#ifdef __cplusplus
}
#endif

#endif
