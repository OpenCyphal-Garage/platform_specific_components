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
    Result messageBuffer_Transmit(std::uint_fast8_t iface_index, std::uint8_t TX_MB_index, const FrameType& frame);

public:
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
    virtual Result write(std::uint_fast8_t interface_index,
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
    virtual Result read(std::uint_fast8_t interface_index,
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
    virtual Result reconfigureFilters(const typename FrameType::Filter* filter_config,
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
    virtual Result select(libuavcan::duration::Monotonic timeout, bool ignore_write_available) override;
};

/**
 * Implementation of the methods from libuavcan's media layer abstracct class InterfaceManager,
 * with the template arguments listed below; for further details of this interface class,
 * refer to the template declaration in libuavcan/media/interface.hpp
 *
 * InterfaceGroupT    = S32K_InterfaceGroup  (previously declared class in the file)
 * InterfaceGroupPtrT = S32K_InterfaceGroup* (raw pointer)
 */
class S32K_InterfaceManager : public InterfaceManager<S32K_InterfaceGroup, S32K_InterfaceGroup*>
{
private:
    /** S32K_InterfaceGroup type object member, which address is used in the factory method next */
    InterfaceGroupType S32K_InterfaceGroupObj_;

public:
    /**
     * Initialize the peripherals needed for the driver in the target MCU, also configures the
     * core clock sources to the Normal RUN profile,
     * @param  filter_config         The filtering to apply equally to all FlexCAN instances.
     * @param  filter_config_length  The length of the @p filter_config argument.
     * @param  out_group             A pointer to set to the started group. This will be nullptr if the start method
     *                               fails.
     * @return libuavcan::Result::Success if the group was successfully started and a valid pointer was returned.
     * @return libuavcan::Result::Failure if the initialization fails at some point.
     *         The caller should assume that @p out_group is an invalid pointer if any failure is returned.
     * @return libuavcan::Result::BadArgument if filter_config_length is out of bound.
     */
    virtual Result startInterfaceGroup(const typename InterfaceGroupType::FrameType::Filter* filter_config,
                                       std::size_t                                           filter_config_length,
                                       InterfaceGroupPtrType&                                out_group) override;

    /**
     * Release and deinitialize the peripherals needed for the current driver, disables all the FlexCAN
     * instances available, waiting for any pending transmission or reception to finish before, also
     * resets the LPIT timer used for timestamping, does not deconfigures the core clock sources
    ¨* configured from startInterfaceGroup neither the pins.
     * @param  inout_group Pointer that will be set to null
     * @return libuavcan::Result::Success. If the used peripherals were deinitialized properly.
     */
    virtual Result stopInterfaceGroup(InterfaceGroupPtrType& inout_group) override;

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
