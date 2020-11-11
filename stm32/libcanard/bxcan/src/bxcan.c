/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2016-2020 UAVCAN Development Team.
/// Authors: Pavel Kirienko <pavel.kirienko@zubax.com>, Tom De Rybel <tom.derybel@robocow.be>

#include "bxcan.h"
#include "bxcan_registers.h"
#include <assert.h>
#include <string.h>

/// Configure the maximum interface index for the bxCAN hardware available in your MCU.
/// Must be set to either 0 (only CAN1) or 1 (CAN1 and CAN2).
#if !defined(BXCAN_MAX_IFACE_INDEX)
#    error "Please set BXCAN_MAX_IFACE_INDEX to the maximum index of the available bxCAN hardware in your MCU."
#endif

/// Configure the system core clock frequency in Hz.
/// Only used by the busy wait in waitMSRINAKBitStateChange().
/// If using official HAL from ST, set to global variable SystemCoreClock
#if defined(USE_HAL_DRIVER) && USE_HAL_DRIVER
extern uint32_t SystemCoreClock;
#    define BXCAN_BUSYWAIT_DELAY_SYSTEM_CORE_CLOCK SystemCoreClock
#else
#    if !defined(BXCAN_BUSYWAIT_DELAY_SYSTEM_CORE_CLOCK)
#        error "Please set BXCAN_BUSYWAIT_DELAY_SYSTEM_CORE_CLOCK to the current system core clock."
#    endif
#endif

/// By default, this macro resolves to the standard assert(). The user can redefine this if necessary.
/// To disable assertion checks completely, make it expand into `(void)(0)`.
#ifndef BXCAN_ASSERT
// Intentional violation of MISRA: assertion macro cannot be replaced with a function definition.
#    define BXCAN_ASSERT(x) assert(x)  // NOSONAR
#endif

/// Trigger an assertion failure if an inner priority inversion is detected at runtime.
/// This setting has no effect if NDEBUG is defined (i.e., assertion checks disabled).
#if !defined(BXCAN_CONFIG_DEBUG_INNER_PRIORITY_INVERSION)
#    define BXCAN_CONFIG_DEBUG_INNER_PRIORITY_INVERSION true
#endif

/// Bit mask for the 29-bit extended ID data frame format.
#define BXCAN_FRAME_EXT_ID_MASK 0x1FFFFFFFU

/// Error trackers, one for each possible CAN interface.
/// As the bxCANReapError() function only returns a bool, we can keep this as a simple flag.
static bool g_error[2] = {false};

/// Time stamps for TX time-out management. There are three TX mailboxes for each CAN interface.
/// An UINT64_MAX value means the time stamp is not set (mailbox is not in use). Zero initialization
/// will work, as the value will be re-set to UINT64_MAX in abortExpiredTxMailboxes() and set to the
/// required value in bxCANPush(). This way, any inadvertently busy mailboxes will still be aborted
/// automatically, making this more robust than initializing to UINT64_MAX from the onset.
static uint64_t g_tx_deadline[(1 + BXCAN_MAX_IFACE_INDEX) * 3] = {0};

/// Converts an extended-ID frame format into the bxCAN TX ID register format.
static uint32_t convertFrameIDToRegister(const uint32_t id)
{
    uint32_t out = 0;

    out = (id & BXCAN_FRAME_EXT_ID_MASK) << 3U;  // Set the 29 EXID bits.
    out |= BXCAN_TIR_IDE;                        // Set the frame identifier extension to extended.

    return out;
}

/// Converts bxCAN TX/RX (sic! both RX/TX are supported) ID register format into the extended-ID frame format.
static uint32_t convertRegisterToFrameID(const uint32_t id)
{
    uint32_t out = 0;

    out = (id >> 3U) & BXCAN_FRAME_EXT_ID_MASK;

    return out;
}

/// Wait for the state change of the MSR INAK bit, with time-out.
static bool waitMSRINAKBitStateChange(volatile const BxCANType* const bxcan_base,  //
                                      const bool                      target_state)
{
    BXCAN_ASSERT((bxcan_base == BXCAN1) || (bxcan_base == BXCAN2));

    bool out_status = false;  // Initialize as failure. It will be set true in case of success.

    // A properly functioning bus will exhibit 11 consecutive recessive bits at the end of every correct transmission,
    // or while the bus is idle. The 11 consecutive recessive bits are made up of:
    //  1 bit - acknowledgement delimiter
    //  7 bit - end of frame bits
    //  3 bit - inter frame space
    // This adds up to 11; therefore, it is not really necessary to wait longer than a few frame TX intervals.
    static const uint16_t TimeoutMilliseconds = 1000;
    for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++)
    {
        const bool state = (bxcan_base->MSR & BXCAN_MSR_INAK) != 0U;
        if (state == target_state)
        {
            out_status = true;  // The MSR INAK bit changed to the required state, exit the loop.
            break;
        }

        // Busy wait one millisecond.
        // With a system core clock value in Hz: scale MHz to kHz (ms) -> / 1000. However, the busy wait
        // loop takes about 7 instruction cycles per tick. Hence: nticks = system core clock (in Hz) / 7000.
        // The counter variable is declared volatile to prevent the compiler from optimizing it away.
        volatile size_t nticks = BXCAN_BUSYWAIT_DELAY_SYSTEM_CORE_CLOCK / 7000U;
        while (--nticks)
        {
        }
    }

    return out_status;
}

/// Detect TX mailbox slots that have expired and abort them.
static void abortExpiredTxMailboxes(volatile BxCANType* const bxcan_base,   //
                                    bool* const               error_iface,  //
                                    const uint64_t            current_time)
{
    BXCAN_ASSERT((bxcan_base == BXCAN1) || (bxcan_base == BXCAN2));              // Valid bxcan base address.
    BXCAN_ASSERT(!((bxcan_base == BXCAN2) && (BXCAN_MAX_IFACE_INDEX == 0)));     // Stay in bounds g_tx_deadline[].
    BXCAN_ASSERT((error_iface == &g_error[0]) || (error_iface == &g_error[1]));  // Valid g_error address.

    // Obtain the busy state of all mailboxes.
    const bool tme[3U] = {(bxcan_base->TSR & BXCAN_TSR_TME0) != 0U,
                          (bxcan_base->TSR & BXCAN_TSR_TME1) != 0U,
                          (bxcan_base->TSR & BXCAN_TSR_TME2) != 0U};

    // Calculate the offset in the deadline table. Three mailboxes per interface, thus offset = 0 or 3.
    const uint8_t iface_index_offset = (bxcan_base == BXCAN1) ? 0 : 3;

    // Check the deadline for each active mailbox.
    for (uint8_t i = 0U; i < 3U; i++)
    {
        // Abort the selected mailbox if it is busy and its deadline has expired. This frees-up stale
        // slots and the timeout error is reported.
        // However, if the slot was not busy, we only reset the deadline to UINT64_MAX, as this only indicates
        // a previously-sent, idle slot that needs its old deadline cleared. This is not an error.
        const uint8_t tx_deadline_idx = i + iface_index_offset;
        if (current_time > g_tx_deadline[tx_deadline_idx])
        {
            g_tx_deadline[tx_deadline_idx] = UINT64_MAX;  // Clear the TX deadline for the selected mailbox.

            // If the slot is active, abort the transmission and report a time-out error.
            if (!tme[i])
            {
                // Abort the selected mailbox.
                if (i == 0U)
                {
                    bxcan_base->TSR |= BXCAN_TSR_ABRQ0;
                }
                if (i == 1U)
                {
                    bxcan_base->TSR |= BXCAN_TSR_ABRQ1;
                }
                if (i == 2U)
                {
                    bxcan_base->TSR |= BXCAN_TSR_ABRQ2;
                }

                *error_iface = true;  // TX timeout error occurred.
            }
        }
    }
}

/// Handle TX/RX errors. Also abort pending TX buffers while in bus-off mode. Errors are reported.
static void processErrorStatus(volatile BxCANType* const bxcan_base,  //
                               bool* const               error_iface)
{
    BXCAN_ASSERT((bxcan_base == BXCAN1) || (bxcan_base == BXCAN2));              // Valid bxcan base address.
    BXCAN_ASSERT(!((bxcan_base == BXCAN2) && (BXCAN_MAX_IFACE_INDEX == 0)));     // Stay in bounds g_tx_deadline[].
    BXCAN_ASSERT((error_iface == &g_error[0]) || (error_iface == &g_error[1]));  // Valid g_error address.

    // Updating error flag.
    const uint8_t lec = (uint8_t)((bxcan_base->ESR & BXCAN_ESR_LEC_MASK) >> BXCAN_ESR_LEC_SHIFT);

    if (lec != 0U)
    {
        bxcan_base->ESR &= ~BXCAN_ESR_LEC_MASK;  // Clear all LEC bits.

        *error_iface = true;  // Update the respective error flag for the selected interface.

        // Abort pending transmissions only if we are in bus-off mode.
        // Also reset the TX mailbox timeouts for the selected interface.
        if (bxcan_base->ESR & BXCAN_ESR_BOFF)
        {
            bxcan_base->TSR |= BXCAN_TSR_ABRQ0 | BXCAN_TSR_ABRQ1 | BXCAN_TSR_ABRQ2;

            const uint8_t iface_index_offset      = (bxcan_base == BXCAN1) ? 0 : 3;
            g_tx_deadline[iface_index_offset]     = UINT64_MAX;
            g_tx_deadline[iface_index_offset + 1] = UINT64_MAX;
            g_tx_deadline[iface_index_offset + 2] = UINT64_MAX;
        }
    }
}

bool bxCANConfigure(const uint8_t      iface_index,  //
                    const BxCANTimings timings,      //
                    const bool         silent)
{
    // Error management and reporting flags.
    bool input_ok   = true;   // Initialize as no errors. Detected errors will set this to false.
    bool out_status = false;  // Initialize as failure. It will be set true in case of success.
    bool inak_ok    = true;   // Initialize as no errors. Detected errors will set this to false.

    // Select the appropriate CAN interface base address and zero the error flags for it.
    // If the interface number is invalid, return with an error.
    volatile BxCANType* bxcan_base   = NULL;  // Selected CAN interface base address.
    uint8_t             filter_index = 0U;
    if (iface_index == 0U)
    {
        bxcan_base   = BXCAN1;
        g_error[0]   = false;
        filter_index = 0U;  // First filter slot for CAN1.
    }
    else if ((iface_index == 1U) && (BXCAN_MAX_IFACE_INDEX == 1U))
    {
        bxcan_base   = BXCAN2;
        g_error[1]   = false;
        filter_index = (uint8_t) BXCAN_NUM_ACCEPTANCE_FILTERS;  // First filter slot for CAN2.
    }
    else
    {
        input_ok = false;  // Invalid CAN interface number.
    }

    // In order to use CAN2, CAN1 must first be initialized as the former depends
    // on the latter. Test if CAN1 is already configured if CAN2 is selected.
    // Note: If only CAN2 is to be used, it is best to configure CAN1 in silent
    // mode and set the acceptance filter to reject everything for CAN1.
    if ((iface_index == 1U) && ((BXCAN1->MCR & BXCAN_MCR_SLEEP) != 0U))
    {
        input_ok = false;  // CAN1 must be initialized before initializing CAN2.
    }

    // Validate the rest of the inputs.
    if ((timings.bit_rate_prescaler < 1U) || (timings.bit_rate_prescaler > 1024U) ||     //
        (timings.max_resync_jump_width < 1U) || (timings.max_resync_jump_width > 4U) ||  //
        (timings.bit_segment_1 < 1U) || (timings.bit_segment_1 > 16U) ||                 //
        (timings.bit_segment_2 < 1U) || (timings.bit_segment_2 > 8U))
    {
        input_ok = false;  // Invalid timings.
    }

    // Initial setup.
    // This is executed step by step, skipping further steps if errors occur during
    // the initialization process.
    if (input_ok)
    {
        // Build an IER mask for all non-reserved bits.
        const uint32_t ier_mask = BXCAN_IER_TMEIE | BXCAN_IER_FMPIE0 | BXCAN_IER_FFIE0 | BXCAN_IER_FOVIE0 |  //
                                  BXCAN_IER_FMPIE1 | BXCAN_IER_FFIE1 | BXCAN_IER_FOVIE1 | BXCAN_IER_EWGIE |  //
                                  BXCAN_IER_EPVIE | BXCAN_IER_BOFIE | BXCAN_IER_LECIE | BXCAN_IER_ERRIE |    //
                                  BXCAN_IER_WKUIE | BXCAN_IER_SLKIE;

        bxcan_base->IER &= ~ier_mask;         // We need no interrupts.
        bxcan_base->MCR &= ~BXCAN_MCR_SLEEP;  // Exit sleep mode.
        bxcan_base->MCR |= BXCAN_MCR_INRQ;    // Request init

        if (!waitMSRINAKBitStateChange(bxcan_base, true))  // Wait for synchronization.
        {
            bxcan_base->MCR |= BXCAN_MCR_RESET;  // Reset the interface, could not synchronize.
            inak_ok = false;                     // INAK was not set.
        }
    }

    if (input_ok && inak_ok)
    {
        // Hardware initialization (the hardware has already confirmed initialization mode, see above)
        bxcan_base->MCR |= BXCAN_MCR_ABOM | BXCAN_MCR_AWUM | BXCAN_MCR_INRQ;  // RM page 648

        // Build a BTR mask for all non-reserved bits.
        const uint32_t btr_mask = BXCAN_BTR_BRP_MASK | BXCAN_BTR_TS1_MASK | BXCAN_BTR_TS2_MASK |  //
                                  BXCAN_BTR_SJW_MASK | BXCAN_BTR_LBKM | BXCAN_BTR_SILM;

        bxcan_base->BTR = (bxcan_base->BTR & ~btr_mask) |                         //
                          (((timings.max_resync_jump_width - 1U) & 3U) << 24U) |  //
                          (((timings.bit_segment_1 - 1U) & 15U) << 16U) |         //
                          (((timings.bit_segment_2 - 1U) & 7U) << 20U) |          //
                          ((timings.bit_rate_prescaler - 1U) & 1023U) |           //
                          (silent ? BXCAN_BTR_SILM : 0U);

        BXCAN_ASSERT(bxcan_base->IER == 0U);  // Make sure the interrupts are indeed disabled.

        bxcan_base->MCR &= ~BXCAN_MCR_INRQ;  // Leave init mode.

        if (!waitMSRINAKBitStateChange(bxcan_base, false))
        {
            bxcan_base->MCR |= BXCAN_MCR_RESET;  // Reset the interface, could not synchronize.
            inak_ok = false;                     // INAK was not cleared.
        }
    }

    if (input_ok && inak_ok)
    {
        // Default filter configuration.
        // For MCUs with two bxCAN interfaces the configuration includes setting the filter start banks in FMR.
        // Note that ALL filters are available ONLY via CAN1! CAN2 filters are offset by 14. We use 14 filters
        // at most always, which simplifies the code and ensures compatibility with all MCU that use bxCAN.
        // For MCUs with one bxCAN interface, there are only 14 filter banks.
        //
        // Filters are alternating between FIFO0 and FIFO1 in order to equalize the load. (Set in FFA1R.)
        // This will cause occasional priority inversion and frame reordering on reception,
        // but that is acceptable for UAVCAN, and a majority of other protocols will tolerate
        // this too, since there will be no reordering within the same CAN ID.
        if (BXCAN_MAX_IFACE_INDEX > 0)
        {
            // MCU with two bxCAN interfaces: configure the filter start banks for each interface.
            // Example MCU: STM32F446.
            // Note: block statement is introduced to contain the scope of fmr (defensive programming).
            {
                uint32_t fmr = BXCAN1->FMR & 0xFFFFC0FEU;
                fmr |= BXCAN_NUM_ACCEPTANCE_FILTERS << 8U;  // CAN2 start bank = 14 (if CAN2 is present)
                BXCAN1->FMR = fmr | BXCAN_FMR_FINIT;        // Set the configuration, enter filter initialization mode.
            }

            BXCAN_ASSERT(((BXCAN1->FMR >> 8U) & 0x3FU) == BXCAN_NUM_ACCEPTANCE_FILTERS);

            BXCAN1->FM1R &= 0xF0000000U;  // Identifier Mask mode. (4 MSB are reserved.)
            BXCAN1->FS1R |= 0x0FFFFFFFU;  // All 32-bit filters. (4 MSB are reserved.)

            BXCAN1->FFA1R =
                (BXCAN1->FFA1R & 0xF0000000U) | 0x0AAAAAAAU;  // Alternate the filters between FIFO0 & FIFO1.
        }
        else
        {
            // MCU with single bxCAN interface has no filter start bank configuration in FMR.
            // Example MCU: STM32L431.
            BXCAN1->FMR |= BXCAN_FMR_FINIT;  // Enter filter initialization mode.

            BXCAN1->FM1R &= 0xFFFFC000U;  // Identifier Mask mode. (18 MSB are reserved.)
            BXCAN1->FS1R |= 0x00003FFFU;  // All 32-bit filters. (18 MSB are reserved.)

            BXCAN1->FFA1R =
                (BXCAN1->FFA1R & 0xFFFFC000U) | 0x00002AAAU;  // Alternate the filters between FIFO0 & FIFO1.
        }

        // Configure one "accept all" filter and enable it.
        BXCAN1->FilterRegister[filter_index].FR1 = 0U;  // Accept all.
        BXCAN1->FilterRegister[filter_index].FR2 = 0U;  // Accept all.
        BXCAN1->FA1R |= (1U << filter_index);           // One filter enabled

        bxcan_base->FMR &= ~BXCAN_FMR_FINIT;  // Leave initialization mode.
        out_status = true;
    }

    return out_status;
}

void bxCANConfigureFilters(const uint8_t           iface_index,  //
                           const BxCANFilterParams params[BXCAN_NUM_ACCEPTANCE_FILTERS])
{
    BXCAN_ASSERT(iface_index <= 1U);  // Unknown interface index.
    BXCAN_ASSERT(params != NULL);

    // Validate the CAN interface index and calculate the filter index offset for the selected interface.
    uint8_t filter_index_offset = 0xFF;
    if (iface_index == 0U)
    {
        filter_index_offset = 0U;
    }
    else if ((iface_index == 1U) && (BXCAN_MAX_IFACE_INDEX == 1U))
    {
        filter_index_offset = (uint8_t)(BXCAN_NUM_ACCEPTANCE_FILTERS);
    }
    else
    {
        filter_index_offset = 0xFF;  // Invalid CAN interface selected.
    }

    // Only modify the registers if the filter register index offset is valid.
    if (filter_index_offset != 0xFF)
    {
        // First we disable all filters. This may cause momentary RX frame losses, but the application
        // should be able to tolerate that. The number of reserved bits is different for devices with
        // one or two bxCAN interfaces.
        if (BXCAN_MAX_IFACE_INDEX > 0)
        {
            BXCAN1->FA1R &= 0xF0000000U;  // Dual CAN: 28 filter banks, 4 MSB reserved.
        }
        else
        {
            BXCAN1->FA1R &= 0xFFFFC000U;  // Single CAN: 14 filter banks, 18 MSB reserved.
        }

        // Having filters disabled we can update the configuration.
        // Register mapping: FR1 - ID, FR2 - Mask
        for (uint8_t i = 0U; i < (uint8_t) BXCAN_NUM_ACCEPTANCE_FILTERS; i++)
        {
            // Converting the ID and the Mask into the representation that can be chewed by the hardware.
            //
            // The logic of the hardware acceptance filters can be described as follows:
            //
            //  accepted = (received_id & filter_mask) == (filter_id & filter_mask)
            //
            // Where:
            //  - accepted      - if true, the frame will be accepted by the filter.
            //  - received_id   - the CAN ID of the received frame, 29-bit extended data frames only.
            //  - filter_id     - the value of the filter ID register.
            //  - filter_mask   - the value of the filter mask register, 29-bit extended data frames only.
            //
            // The following truth table summarizes the logic (where: FM - filter mask, FID - filter ID, RID - received
            // frame ID, A - true if accepted, X - any state):
            //
            //  FM  FID RID A
            //  0   X   X   1
            //  1   0   0   1
            //  1   1   0   0
            //  1   0   1   0
            //  1   1   1   1
            //
            // One would expect that for the purposes of hardware filtering, the special bits should be treated
            // in the same way as the real ID bits. However, this is not the case with bxCAN. The following truth
            // table has been determined empirically (this behavior was not documented as of 2017):
            //
            //  FM  FID RID A
            //  0   0   0   1
            //  0   0   1   0       <-- frame rejected!
            //  0   1   X   1
            //  1   0   0   1
            //  1   1   0   0
            //  1   0   1   0
            //  1   1   1   1
            uint32_t id   = 0;
            uint32_t mask = 0;

            const BxCANFilterParams* const cfg = params + i;

            // Convert the filter to the register format.
            // The special case of a filter entry set to {0, 0} (all bits zero) signifies that the filter should block
            // all traffic. This is done to improve the API, avoiding a magic number. Detect this, and leave that filter
            // disabled to block all traffic as there is no set of filter values that achieves this. Eg: {0, 0x1FFFFFFF}
            // will still allow data with ID = 0 to pass. (Setting a {0, 0} filter in the registers as-such would
            // actually pass all traffic.)
            if ((cfg->extended_id != 0U) ||
                (cfg->extended_mask != 0U))  // Only configure and enable non-{0, 0} filters.
            {
                id   = (cfg->extended_id & BXCAN_FRAME_EXT_ID_MASK) << 3U;
                mask = (cfg->extended_mask & BXCAN_FRAME_EXT_ID_MASK) << 3U;
                id |=
                    BXCAN_RIR_IDE;  // Must be set to accept extended-ID frames. (The mask bit for IDE is already zero.)

                // Applying the converted representation to the registers.
                const uint8_t filter_index               = i + filter_index_offset;
                BXCAN1->FilterRegister[filter_index].FR1 = id;
                BXCAN1->FilterRegister[filter_index].FR2 = mask;

                BXCAN1->FA1R |= (1U << filter_index);  // Enable the filter.
            }
        }
    }
}

bool bxCANReapError(const uint8_t iface_index)
{
    bool out_err = false;

    if (iface_index == 0U)
    {
        out_err    = g_error[0];
        g_error[0] = false;
    }
    else if (iface_index == 1U)
    {
        out_err    = g_error[1];
        g_error[1] = false;
    }
    else
    {
        out_err = true;  // Invalid CAN interface number.
    }

    return out_err;
}

bool bxCANPush(const uint8_t     iface_index,      //
               const uint64_t    current_time,     //
               const uint64_t    deadline,         //
               const uint32_t    extended_can_id,  //
               const size_t      payload_size,     //
               const void* const payload)
{
    // Error management and reporting flags.
    bool input_ok   = true;   // Initialize as no errors. Detected errors will set this to false.
    bool out_status = false;  // Initialize as failure. It will be set true in case of success.

    // Select the appropriate CAN interface base address and error flag.
    // If the interface number is invalid, return with an error.
    volatile BxCANType* bxcan_base  = NULL;
    bool*               error_iface = NULL;
    if (iface_index == 0U)
    {
        bxcan_base  = BXCAN1;
        error_iface = &g_error[0];
    }
    else if ((iface_index == 1U) && (BXCAN_MAX_IFACE_INDEX == 1U))
    {
        bxcan_base  = BXCAN2;
        error_iface = &g_error[1];
    }
    else
    {
        input_ok = false;  // Invalid CAN interface number.
    }

    // Registers any error states for the selected CAN interface. It also handles the requirement
    // to automatically abort all pending transmissions if the CAN controller enters the bus-off state.
    if (input_ok)
    {
        processErrorStatus(bxcan_base, error_iface);
    }

    // Validate the other inputs.
    if (payload_size > 8U)
    {
        input_ok = false;  // Payload size for classic CAN must be <= 8 bytes.
    }

    if ((payload_size > 0U) && (payload == NULL))
    {
        input_ok = false;  // NULL pointer payload with non-zero payload size.
    }

    if (extended_can_id > BXCAN_FRAME_EXT_ID_MASK)
    {
        input_ok = false;  // Extended_can_id must be exactly 29 bits, thus 3 MSB of 32-bit word must be 0.
    }

    // When the frame that should be transmitted is already expired (current_time > deadline), discard it. It would
    // be rejected at reception, and thus causes undue overhead and bus load. When an expired frame is discarded,
    // the TX timeout error is registered, and true is returned (the frame is considered sent).
    if (input_ok && (current_time > deadline))
    {
        *error_iface = true;   // TX timeout error occurred.
        input_ok     = false;  // Skip the further data transmission steps.
        out_status   = true;   // We must return true when discarding the frame. It is considered sent.
    }

    // Seeking an empty slot, checking if priority inversion would occur if we enqueued now.
    // We can always enqueue safely if all TX mailboxes are free and no transmissions are pending.
    uint8_t tx_mailbox  = 0xFF;   // 0xFF indicates no free mailboxes.
    bool    prio_higher = false;  // false indicates no frame with higher priority.
    if (input_ok)
    {
        static const uint32_t AllTME = BXCAN_TSR_TME0 | BXCAN_TSR_TME1 | BXCAN_TSR_TME2;

        if ((bxcan_base->TSR & AllTME) != AllTME)  // At least one TX mailbox is used, detailed check is needed
        {
            // Abort mailboxes with an expired deadline. This frees-up stale
            // slots and the timeout error is reported via the &g_error[] flag.
            abortExpiredTxMailboxes(bxcan_base, error_iface, current_time);

            // Obtain the busy state of all mailboxes, after any stale mailboxes were aborted.
            const bool tme[3U] = {(bxcan_base->TSR & BXCAN_TSR_TME0) != 0U,
                                  (bxcan_base->TSR & BXCAN_TSR_TME1) != 0U,
                                  (bxcan_base->TSR & BXCAN_TSR_TME2) != 0U};

            // Search for free mailboxes.
            // Mailboxes with priority >= the priority of the new frame are detected.
            for (uint8_t i = 0U; i < 3U; i++)
            {
                if (tme[i])  // This TX mailbox is free, we can use it
                {
                    tx_mailbox = i;
                }
                else  // This TX mailbox is pending, check for priority inversion
                {
                    if (extended_can_id >= convertRegisterToFrameID(bxcan_base->TxMailbox[i].TIR))
                    {
                        // There's a mailbox whose priority is higher or equal the priority of the new frame.
                        // Priority inversion would occur! Reject transmission.
                        prio_higher = true;
                    }
                }
            }

            // If no mailboxes are free and none of the mailboxes have higher priority we have a priority inversion.
            if ((tx_mailbox == 0xFF) && (prio_higher == false))
            {
                // All TX mailboxes are busy (this is highly unlikely); at the same time we know that there is no
                // higher or equal priority frame that is currently pending. Therefore, priority inversion has just
                // happend (sic!), because we can't enqueue the higher priority frame due to all TX mailboxes being
                // busy. This scenario is extremely unlikely, because in order for it to happen, the application would
                // need to transmit 4 (four) or more CAN frames with different CAN ID ordered from high ID to low ID
                // nearly at the same time. For example:
                //  1. 0x123        <-- Takes mailbox 0 (or any other)
                //  2. 0x122        <-- Takes mailbox 2 (or any other)
                //  3. 0x121        <-- Takes mailbox 1 (or any other)
                //  4. 0x120        <-- INNER PRIORITY INVERSION HERE (only if all three TX mailboxes are still busy)
                // This situation is even less likely to cause any noticeable disruptions on the CAN bus. Despite that,
                // it is better to warn the developer about that during debugging, so we fire an assertion failure here.
                // It is perfectly safe to remove it.
                // Note: this error condition (all TX mailboxes busy) also occurs when erroneously sending data to the
                // TX mailboxes of a non-existing CAN interface. Eg: accessing CAN2 on a device that only has CAN1.
#if BXCAN_CONFIG_DEBUG_INNER_PRIORITY_INVERSION
                BXCAN_ASSERT(!"CAN PRIO INV");  // NOLINT
#endif
            }
        }
        else  // All TX mailboxes are free, use first
        {
            tx_mailbox = 0U;
        }

        BXCAN_ASSERT(tx_mailbox < 3U);  // Index check - the value must be correct here
    }

    // By this time we've proven that:
    // - The input is valid.                                                        (input_ok    == true)
    // - There are no mailboxes with priority >= the priority of the new frame.     (prio_higher == false)
    // - A priority inversion would not occur.                                      (tx_mailbox  != 0xFF)
    // - A free TX mailbox is available.                                            (tx_mailbox   < 3U)
    // Therefore it is safe to enqueue the frame now.
    if (input_ok && !prio_higher && (tx_mailbox < 3U))
    {
        volatile BxCANTxMailboxType* const mb = &bxcan_base->TxMailbox[tx_mailbox];

        // Set the TX deadline for the selected mailbox.
        g_tx_deadline[tx_mailbox + (iface_index * 3)] = deadline;

        // The payload can be any length up to the MTU. We first make a local copy of the
        // available payload bytes in a zero-filled array of MTU size. This makes the logic
        // for filling the mailbox simpler as the payload pointer can be to variable-size
        // data or even be a NULL pointer.
        uint8_t scratch_data[8] = {0};  // Not strictly needed to zero the storage.
        if (payload_size > 0U)          // The check is needed to avoid calling memcpy() with a NULL pointer.
        {
            // Clang-Tidy raises an error recommending the use of memcpy_s() instead.
            // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
            (void) memcpy(&scratch_data[0], payload, payload_size);  // NOLINT
        }

        // TDTR contains the DLC bits. DLC equals data length except in CAN FD.
        mb->TDTR = (mb->TDTR & ~BXCAN_TDTR_DLC_MASK) | payload_size;

        mb->TDHR = (((uint32_t) scratch_data[7]) << 24U) | (((uint32_t) scratch_data[6]) << 16U) |  //
                   (((uint32_t) scratch_data[5]) << 8U) | (((uint32_t) scratch_data[4]) << 0U);
        mb->TDLR = (((uint32_t) scratch_data[3]) << 24U) | (((uint32_t) scratch_data[2]) << 16U) |  //
                   (((uint32_t) scratch_data[1]) << 8U) | (((uint32_t) scratch_data[0]) << 0U);

        mb->TIR = convertFrameIDToRegister(extended_can_id) | BXCAN_TIR_TXRQ;  // Go.

        // The frame is now enqueued and pending transmission.
        out_status = true;
    }

    return out_status;
}

bool bxCANPop(const uint8_t   iface_index,          //
              uint32_t* const out_extended_can_id,  //
              size_t* const   out_payload_size,     //
              void* const     out_payload)
{
    // Error management and reporting flags.
    bool input_ok   = true;   // Initialize as no errors. Detected errors will set this to false.
    bool out_status = false;  // Initialize as failure. It will be set true in case of success.

    // Select the appropriate CAN interface base address and error flag.
    volatile BxCANType* bxcan_base  = NULL;
    bool*               error_iface = NULL;
    if (iface_index == 0U)
    {
        bxcan_base  = BXCAN1;
        error_iface = &g_error[0];
    }
    else if ((iface_index == 1U) && (BXCAN_MAX_IFACE_INDEX == 1U))
    {
        bxcan_base  = BXCAN2;
        error_iface = &g_error[1];
    }
    else
    {
        input_ok = false;  // Invalid CAN interface number.
    }

    // Validate the output pointers.
    if ((out_extended_can_id == NULL) || (out_payload_size == NULL) || (out_payload == NULL))
    {
        input_ok = false;  // Output pointers can not be NULL.
    }

    // Only try to receive data if the input validation was successful.
    if (input_ok)
    {
        // This function must be polled periodically, so we use this opportunity to do it.
        processErrorStatus(bxcan_base, error_iface);

        volatile uint32_t* const RFxR[2] = {&bxcan_base->RF0R, &bxcan_base->RF1R};

        // Reading the TX FIFO
        for (uint_fast8_t i = 0U; i < 2U; i++)
        {
            volatile BxCANRxMailboxType* const mb = &bxcan_base->RxMailbox[i];

            if (((*RFxR[i]) & BXCAN_RFR_FMP_MASK) != 0U)
            {
                if (*RFxR[i] & BXCAN_RFR_FOVR)
                {
                    *error_iface = true;
                }

                *out_extended_can_id = convertRegisterToFrameID(mb->RIR);

                *out_payload_size = (uint8_t)(mb->RDTR & BXCAN_RDTR_DLC_MASK);

                // Caching to regular (non volatile) memory for faster reads
                const uint32_t rdlr = mb->RDLR;
                const uint32_t rdhr = mb->RDHR;

                uint8_t* out_ptr = (uint8_t*) out_payload;
                *out_ptr++       = (uint8_t)(0xFFU & (rdlr >> 0U));
                *out_ptr++       = (uint8_t)(0xFFU & (rdlr >> 8U));
                *out_ptr++       = (uint8_t)(0xFFU & (rdlr >> 16U));
                *out_ptr++       = (uint8_t)(0xFFU & (rdlr >> 24U));
                *out_ptr++       = (uint8_t)(0xFFU & (rdhr >> 0U));
                *out_ptr++       = (uint8_t)(0xFFU & (rdhr >> 8U));
                *out_ptr++       = (uint8_t)(0xFFU & (rdhr >> 16U));
                *out_ptr++       = (uint8_t)(0xFFU & (rdhr >> 24U));

                // Release FIFO entry we just read
                *RFxR[i] |= BXCAN_RFR_RFOM | BXCAN_RFR_FOVR | BXCAN_RFR_FULL;

                // Only accept the frame if it was a CAN2.0b extended-ID data frame. Standard
                // frames (IDE = 0) (which includes error and overload frames) and remote frames
                // (RTR = 1) are silently dropped. In such cases, we continue the loop to see if
                // there are any more frames available that could be returned instead.
                if (((mb->RIR & BXCAN_RIR_IDE) != 0U) && ((mb->RIR & BXCAN_RIR_RTR) == 0U))
                {
                    // Reading successful, exit the for loop with the received frame ready.
                    out_status = true;
                    break;
                }
            }
        }
    }

    return out_status;
}

bool bxCANComputeTimings(const uint32_t      peripheral_clock_rate,  //
                         const uint32_t      target_bitrate,         //
                         BxCANTimings* const out_timings)
{
    if (target_bitrate < 1000U)
    {
        return false;
    }

    BXCAN_ASSERT(out_timings != NULL);  // NOLINT

    // Clang-Tidy raises an error recommending the use of memset_s() instead.
    // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
    memset(out_timings, 0, sizeof(*out_timings));  // NOLINT

    // Hardware configuration
    static const uint8_t MaxBS1 = 16U;
    static const uint8_t MaxBS2 = 8U;

    // Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
    //      CAN in Automation, 2003
    //
    // According to the source, optimal quanta per bit are:
    //   Bitrate        Optimal Maximum
    //   1000 kbps      8       10
    //   500  kbps      16      17
    //   250  kbps      16      17
    //   125  kbps      16      17
    const uint8_t max_quanta_per_bit = (uint8_t)((target_bitrate >= 1000000) ? 10U : 17U);
    BXCAN_ASSERT(max_quanta_per_bit <= (MaxBS1 + MaxBS2));

    static const uint16_t MaxSamplePointLocationPermill = 900U;

    // Computing (prescaler * BS):
    //   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
    //   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
    // let:
    //   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
    //   PRESCALER_BS = PRESCALER * BS
    // ==>
    //   PRESCALER_BS = PCLK / BITRATE
    const uint32_t prescaler_bs = peripheral_clock_rate / target_bitrate;

    // Searching for such prescaler value so that the number of quanta per bit is highest.
    uint8_t bs1_bs2_sum = (uint8_t)(max_quanta_per_bit - 1U);

    while ((prescaler_bs % (1U + bs1_bs2_sum)) != 0U)
    {
        if (bs1_bs2_sum <= 2U)
        {
            return false;  // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1U + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U))
    {
        return false;  // No solution
    }

    // Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
    // We need to find such values so that the sample point is as close as possible to the optimal value,
    // which is 87.5%, which is 7/8.
    //
    //   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
    //   {{bs2 -> (1 + bs1)/7}}
    //
    // Hence:
    //   bs2 = (1 + bs1) / 7
    //   bs1 = (7 * bs1_bs2_sum - 1) / 8
    //
    // Sample point location can be computed as follows:
    //   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
    //
    // Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
    //   - With rounding to nearest
    //   - With rounding to zero
    uint8_t bs1 = (uint8_t)(((7U * bs1_bs2_sum - 1U) + 4U) / 8U);  // Trying rounding to nearest first
    uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);
    BXCAN_ASSERT(bs1_bs2_sum > bs1);

    {
        const uint16_t sample_point_permill = (uint16_t)(1000U * (1U + bs1) / (1U + bs1 + bs2));

        if (sample_point_permill > MaxSamplePointLocationPermill)  // Strictly more!
        {
            bs1 = (uint8_t)((7U * bs1_bs2_sum - 1U) / 8U);  // Nope, too far; now rounding to zero
            bs2 = (uint8_t)(bs1_bs2_sum - bs1);
        }
    }

    const bool valid = (bs1 >= 1U) && (bs1 <= MaxBS1) && (bs2 >= 1U) && (bs2 <= MaxBS2);

    // Final validation
    // Helpful Python:
    // def sample_point_from_btr(x):
    //     assert 0b0011110010000000111111000000000 & x == 0
    //     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
    //     return (1+ts1+1)/(1+ts1+1+ts2+1)
    if ((target_bitrate != (peripheral_clock_rate / (prescaler * (1U + bs1 + bs2)))) || !valid)
    {
        // This actually means that the algorithm has a logic error, hence assert(0).
        BXCAN_ASSERT(0);  // NOLINT
        return false;
    }

    out_timings->bit_rate_prescaler    = (uint16_t) prescaler;
    out_timings->max_resync_jump_width = 1U;  // One is recommended by DS-015, CANOpen, and DeviceNet
    out_timings->bit_segment_1         = bs1;
    out_timings->bit_segment_2         = bs2;

    return true;
}
