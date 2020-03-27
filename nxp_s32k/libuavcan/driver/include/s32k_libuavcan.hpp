/*
 * Copyright (c) 2020, NXP. All rights reserved.
 * Distributed under The MIT License.
 * Author: Abraham Rodriguez <abraham.rodriguez@nxp.com>
 */

/**
 * @file
 * Header driver file for the media layer of Libuavcan v1 targeting
 * the NXP S32K14 family of automotive grade MCU's running
 * CAN-FD at 4Mbit/s data phase and 1Mbit/s in nominal phase.
 */

#ifndef S32K_LIBUAVCAN_HPP_INCLUDED
#define S32K_LIBUAVCAN_HPP_INCLUDED

/** Driver build configurations file */
#include "build_config.hpp"

/** libuavcan core header files */
#include "libuavcan/media/can.hpp"
#include "libuavcan/media/interfaces.hpp"
#include "libuavcan/platform/memory.hpp"

/** STL queue for the intermediate ISR buffer */
#include <deque>

/** CMSIS Core for __REV macro use */
#include "s32_core_cm4.h"

/**
 * Preprocessor conditionals for deducing the number of CANFD FlexCAN instances in target MCU,
 * this macro is defined inside the desired memory map "S32K14x.h" included header file
 */
#if defined(MCU_S32K142) || defined(MCU_S32K144)
#    define TARGET_S32K_CANFD_COUNT (1u)
#    define DISCARD_COUNT_ARRAY 0

#elif defined(MCU_S32K146)
#    define TARGET_S32K_CANFD_COUNT (2u)
#    define DISCARD_COUNT_ARRAY 0, 0

#elif defined(MCU_S32K148)
#    define TARGET_S32K_CANFD_COUNT (3u)
#    define DISCARD_COUNT_ARRAY 0, 0, 0

#else
#    error "No NXP S32K compatible MCU header file included"
#endif

namespace libuavcan
{
namespace media
{
/**
 * @namespace S32K
 * Microcontroller-specific constants, variables and non-mutating helper functions for the use of the FlexCAN peripheral
 */
namespace S32K
{
/**
 * Helper function for block polling a bit flag until it is set with a timeout of 0.2 seconds using a LPIT timer,
 * the argument list and usage reassembles the classic block polling while loop, and instead of using a third
 * argument to decide if it'ss a timed block for a clear or set, the two flavors of the function are provided.
 *
 * @param  flagRegister Register where the flag is located.
 * @param  flagMask     Mask to AND'nd with the register for isolating the flag.
 * @return libuavcan::Result::Success If the flag set before the timeout expiration..
 * @return libuavcan::Result::Failure If a timeout ocurred before the desired flag set.
 */
libuavcan::Result flagPollTimeout_Set(volatile std::uint32_t& flagRegister, std::uint32_t flag_Mask);

/**
 * Helper function for block polling a bit flag until it is cleared with a timeout of 0.2 seconds using a LPIT timer
 *
 * @param  flagRegister Register where the flag is located.
 * @param  flagMask     Mask to AND'nd with the register for isolating the flag.
 * @return libuavcan::Result::Success If the flag cleared before the timeout expiration..
 * @return libuavcan::Result::Failure If a timeout ocurred before the desired flag cleared.
 */
libuavcan::Result flagPollTimeout_Clear(volatile std::uint32_t& flagRegister, std::uint32_t flag_Mask);

}  // END namespace S32K

/**
 * Implementation of the methods from libuavcan's media layer abstracct class InterfaceGroup,
 * with the template arguments listed below; for further details of this interface class,
 * refer to the template declaration in libuavcan/media/interface.hpp
 *
 * FrameT = Frame: MTUBytesParam = MaxFrameSizeBytes (64 bytes for CAN-FD)
 *                 FlagBitsCompareMask = 0x00 (default)
 * MaxTxFrames = 1 (default)
 * MaxRxFrames = 1 (default)
 */
class S32K_InterfaceGroup : public InterfaceGroup<CAN::Frame<CAN::TypeFD::MaxFrameSizeBytes>>
{
private:
    /**
     * Helper function for an immediate transmission through an available message buffer
     *
     * @param  TX_MB_index The index from an already polled available message buffer.
     * @param  frame       The individual frame being transmitted.
     * @return libuavcan::Result:Success after a successful transmission request.
     */
    libuavcan::Result messageBuffer_Transmit(std::uint_fast8_t iface_index,
                                             std::uint8_t      TX_MB_index,
                                             const FrameType&  frame);

    /**
     * Helper function for resolving the timestamp of a received frame from FlexCAN'S 16-bit overflowing timer. Based
     * on Pyuavcan's SourceTimeResolver class from which the terms source and target are used.
     * Note: A maximum of 820 microseconds is allowed for the reception ISR to reach this function starting from
     *       a successful frame reception. The computation relies in that no more than a full period from the 16-bit
     *       timestamping timer running at 80Mhz have passed, this could occur in deadlocks or priority inversion
     *       scenarios since 820 uSecs constitute a significant amount of cycles, if this happens, timestamps would stop
     *       being monotonic.
     * @param  frame_timestamp Source clock read from the FlexCAN's peripheral timer.
     * @param  instance        The interface instance number used by the ISR
     * @return time::Monotonic 64-bit timestamp resolved from 16-bit Flexcan's timer samples.
     */
    static libuavcan::time::Monotonic resolve_Timestamp(std::uint64_t frame_timestamp, std::uint8_t instance);

public:
    /**
     * FlexCAN ISR for frame reception, implements a walkaround to the S32K1 FlexCAN's lack of a RX FIFO neither a DMA
     * triggering mechanism for CAN-FD frames in hardware. Completes in at max 7472 cycles when compiled with g++ at -O3. 
     * @param instance The FlexCAN peripheral instance number in which the ISR will be executed, starts at 0.
     *                 differing form this library's interface indexes that start at 1.
     */
    static void S32K_libuavcan_ISR_handler(std::uint8_t instance);

    /**
     * Get the number of CAN-FD capable FlexCAN modules in current S32K14 MCU
     * @return 1-* depending of the target MCU.
     */
    virtual std::uint_fast8_t getInterfaceCount() const override;

    /**
     * Send a frame through a particular available FlexCAN instance
     * @param  interface_index  The index of the interface in the group to write the frames to.
     * @param  frames           1..MaxTxFrames frames to write into the system queues for immediate transmission.
     * @param  frames_len       The number of frames in the frames array that should be sent
     *                          (starting from frame 0).
     * @param  out_frames_written
     *                          Will return MaxTxFrames in current implementation if the frame was sent successfully
     * @return libuavcan::Result::Success if all frames were written.
     * @return libuavcan::Result::BadArgument if interface_index or frames_len are out of bound.
     */
    virtual libuavcan::Result write(std::uint_fast8_t interface_index,
                                    const FrameType (&frames)[TxFramesLen],
                                    std::size_t  frames_len,
                                    std::size_t& out_frames_written) override;

    /**
     * Read from an intermediate ISR Frame buffer of an FlexCAN instance.
     * @param  interface_index  The index of the interface in the group to read the frames from.
     * @param  out_frames       A buffer of frames to read.
     * @param  out_frames_read  On output the number of frames read into the out_frames array.
     * @return libuavcan::Result::Success If no errors occurred.
     * @return libuavcan::Result::BadArgument If interface_index is out of bound.
     */
    virtual libuavcan::Result read(std::uint_fast8_t interface_index,
                                   FrameType (&out_frames)[RxFramesLen],
                                   std::size_t& out_frames_read) override;

    /**
     * Reconfigure reception filters for dynamic subscription of nodes, all the previous filter configurations are
     * cleared.
     * @param  filter_config         The filtering to apply equally to all members of the group.
     * @param  filter_config_length  The length of the @p filter_config argument.
     * @return libuavcan::Result::Success if the group's receive filtering was successfully reconfigured.
     * @return libuavcan::Result::Failure if a register didn't get configured as desired.
     * @return libuavcan::Result::BadArgument if filter_config_length is out of bound.
     */
    virtual libuavcan::Result reconfigureFilters(const typename FrameType::Filter* filter_config,
                                                 std::size_t                       filter_config_length) override;

    /**
     * Block with timeout for available Message buffers.
     * @param [in]     timeout                  The amount of time to wait for and available message buffer.
     * @param [in]     ignore_write_available   If set to true, will check availability only for RX MB's
     *
     * @return  libuavcan::Result::SuccessTimeout if timeout occurred and no required MB's became available.
     *          libuavcan::Result::Success if an interface is ready for read, and if
     *          @p ignore_write_available is false, or write.
     */
    virtual libuavcan::Result select(libuavcan::duration::Monotonic timeout, bool ignore_write_available) override;
};

/**
 * Implementation of the methods from libuavcan's media layer abstracct class InterfaceManager,
 * with the template arguments listed below; for further details of this interface class,
 * refer to the template declaration in libuavcan/media/interface.hpp
 *
 * InterfaceGroupT    = S32K_InterfaceGroup  (previously declared class in the file)
 * InterfaceGroupPtrT = S32K_InterfaceGroup* (raw pointer)
 */
class S32K_InterfaceManager : private InterfaceManager<S32K_InterfaceGroup, S32K_InterfaceGroup*>
{
private:
    /** S32K_InterfaceGroup type object member, which address is used in the factory method next */
    InterfaceGroupType S32K_InterfaceGroupObj_;

public:
    /**
     * Initializes the peripherals needed for libuavcan driver layer in current MCU
     * @param  filter_config         The filtering to apply equally to all FlexCAN instances.
     * @param  filter_config_length  The length of the @p filter_config argument.
     * @param  out_group             A pointer to set to the started group. This will be nullptr if the start method
     *                               fails.
     * @return libuavcan::Result::Success if the group was successfully started and a valid pointer was returned.
     * @return libuavcan::Result::Failure if the initialization fails at some point.
     *         The caller should assume that @p out_group is an invalid pointer if any failure is returned.
     * @return libuavcan::Result::BadArgument if filter_config_length is out of bound.
     */
    virtual libuavcan::Result startInterfaceGroup(const typename InterfaceGroupType::FrameType::Filter* filter_config,
                                                  std::size_t            filter_config_length,
                                                  InterfaceGroupPtrType& out_group) override;

    /**
     * Deinitializes the peripherals needed for the current libuavcan driver layer.
     * @param inout_group Pointer that will be set to null
     * @return libuavcan::Result::Success. If the used peripherals were deinitialized properly.
     */
    virtual libuavcan::Result stopInterfaceGroup(InterfaceGroupPtrType& inout_group) override;
    /**
     * Return the number of filters that the current UAVCAN node can support.
     * @return The maximum number of frame filters available for filter groups managed by this object,
     *         i.e. the number of combinations of ID and mask that each FlexCAN instance supports
     */
    virtual std::size_t getMaxFrameFilters() const override;
};

}  // END namespace media
}  // END namespace libuavcan

#endif  // S32K_LIBUAVCAN_HPP_INCLUDED
