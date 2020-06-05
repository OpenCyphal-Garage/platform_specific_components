/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2020 UAVCAN Development Team.
/// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

// This is needed to enable the necessary declarations in sys/
#ifndef _GNU_SOURCE
#    define _GNU_SOURCE
#endif

#include "socketcan.h"

#ifdef __linux__
#    include <linux/can.h>
#    include <linux/can/raw.h>
#    include <net/if.h>
#    include <sys/ioctl.h>
#else
#    error "Unsupported OS -- feel free to add support for your OS here. " \
        "Zephyr and NuttX are known to support the SocketCAN API."
#endif

#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <time.h>

#define KILO 1000L
#define MEGA (KILO * KILO)

static int16_t getNegatedErrno()
{
    const int out = -abs(errno);
    if (out < 0)
    {
        if (out >= INT16_MIN)
        {
            return (int16_t) out;
        }
    }
    else
    {
        assert(false);  // Requested an error when errno is zero?
    }
    return INT16_MIN;
}

static int16_t doPoll(const SocketCANFD fd, const int16_t mask, const CanardMicrosecond timeout_usec)
{
    struct pollfd fds;
    memset(&fds, 0, sizeof(fds));
    fds.fd     = fd;
    fds.events = mask;

    struct timespec ts;
    ts.tv_sec  = (long) (timeout_usec / (CanardMicrosecond) MEGA);
    ts.tv_nsec = (long) (timeout_usec % (CanardMicrosecond) MEGA) * KILO;

    const int poll_result = ppoll(&fds, 1, &ts, NULL);
    if (poll_result < 0)
    {
        return getNegatedErrno();
    }
    if (poll_result == 0)
    {
        return 0;
    }
    if (((uint32_t) fds.revents & (uint32_t) mask) == 0)
    {
        return -EIO;
    }

    return 1;
}

SocketCANFD socketcanOpen(const char* const iface_name, const bool can_fd)
{
    const size_t iface_name_size = strlen(iface_name) + 1;
    if (iface_name_size > IFNAMSIZ)
    {
        return -ENAMETOOLONG;
    }

    const int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);  // NOLINT
    bool      ok = fd >= 0;

    if (ok)
    {
        struct ifreq ifr;
        (void) memset(&ifr, 0, sizeof(ifr));
        (void) memcpy(ifr.ifr_name, iface_name, iface_name_size);
        ok = 0 == ioctl(fd, SIOCGIFINDEX, &ifr);
        if (ok)
        {
            struct sockaddr_can addr;
            (void) memset(&addr, 0, sizeof(addr));
            addr.can_family  = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            ok               = 0 == bind(fd, (struct sockaddr*) &addr, sizeof(addr));
        }
    }

    if (ok && can_fd)
    {
        const int en = 1;
        ok           = 0 == setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &en, sizeof(en));
    }

    if (ok)
    {
        return fd;
    }

    (void) close(fd);
    return getNegatedErrno();
}

int16_t socketcanPush(const SocketCANFD fd, const CanardFrame* const frame, const CanardMicrosecond timeout_usec)
{
    if ((frame == NULL) || (frame->payload == NULL) || (frame->payload_size > UINT8_MAX))
    {
        return -EINVAL;
    }

    const int16_t poll_result = doPoll(fd, POLLOUT, timeout_usec);
    if (poll_result > 0)
    {
        // We use the CAN FD struct regardless of whether the CAN FD socket option is set.
        // Per the user manual, this is acceptable because they are binary compatible.
        struct canfd_frame cfd;
        (void) memset(&cfd, 0, sizeof(cfd));
        cfd.can_id = frame->extended_can_id | CAN_EFF_FLAG;
        cfd.len    = (uint8_t) frame->payload_size;
        // We set the bit rate switch on the assumption that it will be ignored by non-CAN-FD-capable hardware.
        cfd.flags = CANFD_BRS;
        (void) memcpy(cfd.data, frame->payload, frame->payload_size);

        // If the payload is small, use the smaller MTU for compatibility with non-FD sockets.
        // This way, if the user attempts to transmit a CAN FD frame without having the CAN FD socket option enabled,
        // an error will be triggered here.  This is convenient -- we can handle both FD and Classic CAN uniformly.
        const size_t mtu = (frame->payload_size > CAN_MAX_DLEN) ? CANFD_MTU : CAN_MTU;
        if (write(fd, &cfd, mtu) < 0)
        {
            return getNegatedErrno();
        }
    }
    return poll_result;
}

int16_t socketcanPop(const SocketCANFD       fd,
                     CanardFrame* const      out_frame,
                     const size_t            payload_buffer_size,
                     void* const             payload_buffer,
                     const CanardMicrosecond timeout_usec)
{
    if ((out_frame == NULL) || (payload_buffer == NULL))
    {
        return -EINVAL;
    }

    const int16_t poll_result = doPoll(fd, POLLIN, timeout_usec);
    if (poll_result > 0)
    {
        struct timespec ts;
        if (clock_gettime(CLOCK_TAI, &ts) < 0)  // TODO: request the timestamp from the kernel.
        {
            return getNegatedErrno();
        }

        // We use the CAN FD struct regardless of whether the CAN FD socket option is set.
        // Per the user manual, this is acceptable because they are binary compatible.
        struct canfd_frame cfd;
        const ssize_t      read_size = read(fd, &cfd, sizeof(cfd));
        if (read_size < 0)
        {
            return getNegatedErrno();
        }
        if ((read_size != CAN_MTU) && (read_size != CANFD_MTU))
        {
            return -EIO;
        }
        if (cfd.len > payload_buffer_size)
        {
            return -EFBIG;
        }

        const bool valid = ((cfd.can_id & CAN_EFF_FLAG) != 0) &&  // Extended frame
                           ((cfd.can_id & CAN_RTR_FLAG) == 0) &&  // Not RTR frame
                           ((cfd.can_id & CAN_ERR_FLAG) == 0);    // Not error frame
        if (!valid)
        {
            return 0;  // Not an extended data frame -- drop silently and return early.
        }

        (void) memset(out_frame, 0, sizeof(CanardFrame));
        out_frame->timestamp_usec  = (CanardMicrosecond)((ts.tv_sec * MEGA) + (ts.tv_nsec / KILO));
        out_frame->extended_can_id = cfd.can_id & CAN_EFF_MASK;
        out_frame->payload_size    = cfd.len;
        out_frame->payload         = payload_buffer;
        (void) memcpy(payload_buffer, &cfd.data[0], cfd.len);
    }
    return poll_result;
}

int16_t socketcanFilter(const SocketCANFD fd, const size_t num_configs, const SocketCANFilterConfig* const configs)
{
    if (configs == NULL)
    {
        return -EINVAL;
    }
    if (num_configs > CAN_RAW_FILTER_MAX)
    {
        return -EFBIG;
    }

    struct can_filter cfs[CAN_RAW_FILTER_MAX];
    for (size_t i = 0; i < num_configs; i++)
    {
        cfs[i].can_id   = (configs[i].extended_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
        cfs[i].can_mask = (configs[i].mask & CAN_EFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;
    }

    const int ret =
        setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, cfs, (socklen_t)(sizeof(struct can_filter) * num_configs));

    return (ret < 0) ? getNegatedErrno() : 0;
}
