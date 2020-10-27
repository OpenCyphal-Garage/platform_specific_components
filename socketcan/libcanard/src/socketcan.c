/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2020 UAVCAN Development Team.
/// Authors: Pavel Kirienko <pavel.kirienko@zubax.com>, Tom De Rybel <tom.derybel@robocow.be>

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
#    include <sys/socket.h>
#else
#    error "Unsupported OS -- feel free to add support for your OS here. " \
        "Zephyr and NuttX are known to support the SocketCAN API."
#endif

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

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

    // Enable CAN FD if required.
    if (ok && can_fd)
    {
        const int en = 1;
        ok           = 0 == setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &en, sizeof(en));
    }

    // Enable timestamping.
    if (ok)
    {
        const int en = 1;
        ok           = 0 == setsockopt(fd, SOL_SOCKET, SO_TIMESTAMP, &en, sizeof(en));
    }

    // Enable outgoing-frame loop-back.
    if (ok)
    {
        const int en = 1;
        ok           = 0 == setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &en, sizeof(en));
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
                     const CanardMicrosecond timeout_usec,
                     bool* const             loopback)
{
    if ((out_frame == NULL) || (payload_buffer == NULL))
    {
        return -EINVAL;
    }

    const int16_t poll_result = doPoll(fd, POLLIN, timeout_usec);
    if (poll_result > 0)
    {
        // Initialize the message header scatter/gather array.
        // We use the CAN FD struct regardless of whether the CAN FD socket option is set.
        // Per the user manual, this is acceptable because they are binary compatible.
        struct iovec       iov           = {0};                    // Scatter/gather array items struct.
        struct canfd_frame sockcan_frame = {0};                    // CAN FD frame storage.
        iov.iov_base                     = &sockcan_frame;         // Starting address.
        iov.iov_len                      = sizeof(sockcan_frame);  // Number of bytes to transfer.

        // Determine the size of the ancillary data and zero-initialize the buffer for it.
        // We require space for both the receive message and the time stamp.
        // TODO: Aligned storage is used. --> Don't know how to do this in a portable way....
        // TODO: C complains that the control_size is not known at compile time, hence the use of malloc/free.
        //       Should not be needed, as those structs are fixed-size?? Ah, cmsghdr has a flexarray extension in c99?
        const size_t control_size = sizeof(struct cmsghdr) + sizeof(struct timeval);
        uint8_t*     control      = (uint8_t*) malloc(control_size);  // TODO: try to get rig of malloc!
        if (control == NULL)
        {
            return -ENOMEM;
        }
        (void) memset(control, 0, control_size);

        // Initialize the message header used by recvmsg.
        struct msghdr msg  = {0};           // Message header struct.
        msg.msg_iov        = &iov;          // Scatter/gather array.
        msg.msg_iovlen     = 1;             // Number of elements in the scatter/gather array.
        msg.msg_control    = control;       // Ancillary data.
        msg.msg_controllen = control_size;  // Ancillary data buffer length.

        // Non-blocking receive messages from the socket and validate.
        const ssize_t read_size = recvmsg(fd, &msg, MSG_DONTWAIT);
        if (read_size <= 0)
        {
            const int neg_errno = getNegatedErrno();

            // Return either the negated error code, or zero in case the socket is marked nonblocking and the receive
            // operation would block. POSIX.1-2001 allows either EWOULDBLOCK or EAGAIN to be returned for this case,
            // and does not require these constants to have the same value. Thus, we check for both possibilities.
            free(control);
            return (read_size < 0 && (neg_errno == -EWOULDBLOCK || neg_errno == -EAGAIN)) ? 0 : neg_errno;
        }
        if ((read_size != CAN_MTU) && (read_size != CANFD_MTU))
        {
            free(control);
            return -EIO;
        }
        if (sockcan_frame.len > payload_buffer_size)
        {
            free(control);
            return -EFBIG;
        }

        const bool valid = ((sockcan_frame.can_id & CAN_EFF_FLAG) != 0) &&  // Extended frame
                           ((sockcan_frame.can_id & CAN_ERR_FLAG) == 0) &&  // Not RTR frame
                           ((sockcan_frame.can_id & CAN_RTR_FLAG) == 0);    // Not error frame
        if (!valid)
        {
            free(control);
            return 0;  // Not an extended data frame -- drop silently and return early.
        }

        const bool loopback_frame = ((uint32_t) msg.msg_flags & (uint32_t) MSG_CONFIRM) != 0;  // NOLINT
        if (loopback == NULL && loopback_frame)
        {
            free(control);
            return 0;  // The loopback pointer is NULL and this is a loopback frame -- drop silently and return early.
        }
        if (loopback != NULL)
        {
            *loopback = loopback_frame;
        }

        // Obtain the CAN frame TAI time stamp from the kernel.
        // Comparing to clock_gettime(CLOCK_TAI, &ts) proved that the time stamp is from the desired, monotonous
        // TAI clock source.
        // TODO: should I keep these assertion checks?
        const struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
        struct timeval        tv   = {0};
        assert(cmsg != NULL);
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP)
        {
            (void) memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));  // Copy to avoid alignment problems
            assert(tv.tv_sec >= 0 && tv.tv_usec >= 0);
        }
        else
        {
            assert(0);
            free(control);
            return -EIO;
        }

        (void) memset(out_frame, 0, sizeof(CanardFrame));
        out_frame->timestamp_usec  = (CanardMicrosecond)(((uint64_t) tv.tv_sec * MEGA) + tv.tv_usec);
        out_frame->extended_can_id = sockcan_frame.can_id & CAN_EFF_MASK;
        out_frame->payload_size    = sockcan_frame.len;
        out_frame->payload         = payload_buffer;
        (void) memcpy(payload_buffer, &sockcan_frame.data[0], sockcan_frame.len);

        free(control);
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
