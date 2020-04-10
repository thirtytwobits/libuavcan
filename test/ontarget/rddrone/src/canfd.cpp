/*
 * Copyright (c) 2020, NXP. All rights reserved.
 * Distributed under The MIT License.
 * Author: Abraham Rodriguez <abraham.rodriguez@nxp.com>
 */

/*
 * Source driver file for the media layer of Libuavcan v1 targeting
 * the NXP S32K14 family of automotive grade MCU's running
 * CAN-FD at 4Mbit/s data phase and 1Mbit/s in nominal phase.
 */

/*
 * Macro for additional configuration needed when using a TJA1044 transceiver, which is used
 * in NXP's UCANS32K146 board, set to 0 when using EVB's or other boards.
 */
#ifndef UAVCAN_NODE_BOARD_USED
#    define UAVCAN_NODE_BOARD_USED 1
#endif

#if defined(__GNUC__)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wold-style-cast"
#    pragma GCC diagnostic ignored "-Wcast-align"
#endif

/* S32K driver header file */
#include "libuavcan/media/S32K/canfd.hpp"

/* STL queue for the intermediate ISR buffer */
#include <deque>
#include <type_traits>

/* libuavcan core header file for static pool allocator */
#include "libuavcan/platform/memory.hpp"

/* CMSIS Core for __REV macro use */
#include "s32_core_cm4.h"

/*
 * Include desired target S32K14x memory map header file dependency,
 * defaults to S32K146 from NXP's UCANS32K146 board
 */
#include "S32K146.h"

/*
 * Preprocessor conditionals for deducing the number of CANFD FlexCAN instances in target MCU,
 * this macro is defined inside the desired memory map "S32K14x.h" included header file
 */
#if defined(MCU_S32K142) || defined(MCU_S32K144)
#    define TARGET_S32K_CANFD_COUNT (1u)

#elif defined(MCU_S32K146)
#    define TARGET_S32K_CANFD_COUNT (2u)

#elif defined(MCU_S32K148)
#    define TARGET_S32K_CANFD_COUNT (3u)

#else
#    error "No NXP S32K compatible MCU header file included"
#endif

// +--------------------------------------------------------------------------+
// | PRIVATE IMPLEMENTATION AND STATIC STORAGE
// +--------------------------------------------------------------------------+

namespace libuavcan
{
namespace media
{
namespace S32K
{
namespace
{
/* Tunable frame capacity for the ISR reception FIFO, each frame adds 80 bytes of required .bss memory */
constexpr static std::size_t Frame_Capacity = 40u;

/* Number of filters supported by a single FlexCAN instance */
constexpr static unsigned int Filter_Count = 5u;

/* Lookup table for NVIC IRQ numbers for each FlexCAN instance */
constexpr static std::uint32_t FlexCAN_NVIC_Indices[][2u] = {{2u, 0x20000}, {2u, 0x1000000}, {2u, 0x80000000}};

/* Array of each FlexCAN instance's addresses for dereferencing from */
constexpr static CAN_Type* FlexCAN[] = CAN_BASE_PTRS;

/* Lookup table for FlexCAN indices in PCC register */
constexpr static unsigned int PCC_FlexCAN_Index[] = {36u, 37u, 43u};

/* Size in words (4 bytes) of the offset between the location of message buffers in FlexCAN's dedicated RAM */
constexpr static unsigned int MB_Size_Words = 18u;

/* Offset in words for reaching the payload of a message buffer */
constexpr static unsigned int MB_Data_Offset = 2u;

/*
 * Enumeration for converting from a bit number to an index, used for some registers where a bit flag for a nth
 * message buffer is represented as a bit left shifted nth times. e.g. 2nd MB is 0b100 = 4 = (1 << 2)
 */
enum MB_bit_to_index : unsigned int
{
    MessageBuffer0 = 0x1,  /* Number representing the bit for the zeroth MB (1 << 2) */
    MessageBuffer1 = 0x2,  /* Number for the bit of the first  MB (1 << 3) */
    MessageBuffer2 = 0x4,  /* Number for the bit of the second MB (1 << 2) */
    MessageBuffer3 = 0x8,  /* Number for the bit of the third  MB (1 << 3) */
    MessageBuffer4 = 0x10, /* Number for the bit of the fourth MB (1 << 4) */
    MessageBuffer5 = 0x20, /* Number for the bit of the fifth  MB (1 << 5) */
    MessageBuffer6 = 0x40, /* Number for the bit of the sixth  MB (1 << 6) */
};

// +--------------------------------------------------------------------------+
// | S32KFlexCan
// +--------------------------------------------------------------------------+

/**
 * Per-interface implementation.
 */
class S32KFlexCan final
{
public:
    S32KFlexCan(unsigned peripheral_index)
        : index_(peripheral_index)
        , fc_(FlexCAN[index_])
        , data_ISR_word_{0}
        , discarded_frames_count_(0)
        , frame_ISRbuffer_()
    {}

    ~S32KFlexCan() = default;

    S32KFlexCan(const S32KFlexCan&) = delete;
    S32KFlexCan& operator=(const S32KFlexCan&) = delete;
    S32KFlexCan(S32KFlexCan&&)                 = delete;
    S32KFlexCan& operator=(S32KFlexCan&&) = delete;

    /**
     * Configure and start the interface.
     */
    Result start(const typename InterfaceManager::InterfaceGroupType::FrameType::Filter* filter_config,
                 std::size_t                                                             filter_config_length)
    {
        /* FlexCAN instance initialization */
        PCC->PCCn[PCC_FlexCAN_Index[index_]] = PCC_PCCn_CGC_MASK; /* FlexCAN clock gating */
        fc_->MCR |= CAN_MCR_MDIS_MASK;        /* Disable FlexCAN module for clock source selection */
        fc_->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK; /* Clear any previous clock source configuration */
        fc_->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;  /* Select bus clock as source (40Mhz)*/

        enter_freeze_mode();

        /* Next configurations are only permitted in freeze mode */
        fc_->MCR |= CAN_MCR_FDEN_MASK |          /* Habilitate CANFD feature */
                    CAN_MCR_FRZ_MASK;            /* Enable freeze mode entry when HALT bit is asserted */
        fc_->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK; /* Activate the use of ISO 11898-1 CAN-FD standard */
        if (index_ == 0)
        {
            fc_->CTRL2 |= CAN_CTRL2_TIMER_SRC_MASK;
        }

        /* CAN Bit Timing (CBT) configuration for a nominal phase of 1 Mbit/s with 80 time quantas,
            in accordance with Bosch 2012 specification, sample point at 83.75% */
        fc_->CBT |= CAN_CBT_BTF_MASK |     /* Enable extended bit timing configurations for CAN-FD for
                                              setting up separately nominal and data phase */
                    CAN_CBT_EPRESDIV(0) |  /* Prescaler divisor factor of 1 */
                    CAN_CBT_EPROPSEG(46) | /* Propagation segment of 47 time quantas */
                    CAN_CBT_EPSEG1(18) |   /* Phase buffer segment 1 of 19 time quantas */
                    CAN_CBT_EPSEG2(12) |   /* Phase buffer segment 2 of 13 time quantas */
                    CAN_CBT_ERJW(12);      /* Resynchronization jump width same as PSEG2 */

        /* CAN-FD Bit Timing (FDCBT) for a data phase of 4 Mbit/s with 20 time quantas,
            in accordance with Bosch 2012 specification, sample point at 75% */
        fc_->FDCBT |= CAN_FDCBT_FPRESDIV(0) | /* Prescaler divisor factor of 1 */
                      CAN_FDCBT_FPROPSEG(7) | /* Propagation segment of 7 time quantas
                                                 (only register that doesn't add 1) */
                      CAN_FDCBT_FPSEG1(6) |   /* Phase buffer segment 1 of 7 time quantas */
                      CAN_FDCBT_FPSEG2(4) |   /* Phase buffer segment 2 of 5 time quantas */
                      CAN_FDCBT_FRJW(4);      /* Resynchorinzation jump width same as PSEG2 */

        /* Additional CAN-FD configurations */
        fc_->FDCTRL |= CAN_FDCTRL_FDRATE_MASK | /* Enable bit rate switch in data phase of frame */
                       CAN_FDCTRL_TDCEN_MASK |  /* Enable transceiver delay compensation */
                       CAN_FDCTRL_TDCOFF(5) |   /* Setup 5 cycles for data phase sampling delay */
                       CAN_FDCTRL_MBDSR0(3);    /* Setup 64 bytes per message buffer (7 MB's) */

        /* Message buffers are located in a dedicated RAM inside FlexCAN, they aren't affected by reset,
         * so they must be explicitly initialized, they total 128 slots of 4 words each, which sum
         * to 512 bytes, each MB is 72 byte in size ( 64 payload and 8 for headers )
         */
        for (std::uint8_t j = 0; j < CAN_RAMn_COUNT; j++)
        {
            fc_->RAMn[j] = 0;
        }

        /* Clear the reception masks before configuring the ones needed */
        for (std::uint8_t j = 0; j < CAN_RXIMR_COUNT; j++)
        {
            fc_->RXIMR[j] = 0;
        }

        /* Setup maximum number of message buffers as 7, 0th and 1st for transmission and 2nd-6th for RX */
        fc_->MCR &= ~CAN_MCR_MAXMB_MASK;                     /* Clear previous configuration of MAXMB, default is 0xF */
        fc_->MCR |= CAN_MCR_MAXMB(6) | CAN_MCR_SRXDIS_MASK | /* Disable self-reception of frames if ID matches */
                    CAN_MCR_IRMQ_MASK;                       /* Enable individual message buffer masking */

        /* Enable interrupt in NVIC for FlexCAN reception with default priority (ID = 81) */
        S32_NVIC->ISER[FlexCAN_NVIC_Indices[index_][0]] = FlexCAN_NVIC_Indices[index_][1];

        /* Enable interrupts of reception MB's (0b1111100) */
        fc_->IMASK1 = CAN_IMASK1_BUF31TO0M(124);

        return reconfigureFilters(filter_config, filter_config_length);
    }

    void isrHandler()
    {
        /* Initialize variable for finding which MB received */
        std::uint8_t mb_index = 0;

        /* Check which RX MB caused the interrupt (0b1111100) mask for 2nd-6th MB */
        switch (fc_->IFLAG1 & 124)
        {
        case MessageBuffer2:
            mb_index = 2u; /* Case for 2nd MB */
            break;
        case MessageBuffer3:
            mb_index = 3u; /* Case for 3th MB */
            break;
        case MessageBuffer4:
            mb_index = 4u; /* Case for 4th MB */
            break;
        case MessageBuffer5:
            mb_index = 5u; /* Case for 5th MB */
            break;
        case MessageBuffer6:
            mb_index = 6u; /* Case for 6th MB */
            break;
        default:
            // This should never happen.
            return;
        }

        /* Receive a frame only if the buffer its under its capacity */
        if (frame_ISRbuffer_.size() <= Frame_Capacity)
        {
            // TODO: use fixed FIFO buffers that are lockless instead of deque
            // TODO: don't byte-swap in the ISR. Defer this until read.
            // TODO: See if we can set the timestamp to be ch1 of lpit 1.

            /* Harvest the Message buffer, read of the control and status word locks the MB */

            /* Get the raw DLC from the message buffer that received a frame */
            std::uint32_t dlc_ISR_raw =
                ((fc_->RAMn[mb_index * MB_Size_Words]) & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;

            /* Create CAN::FrameDLC type variable from the raw dlc */
            CAN::FrameDLC dlc_ISR = CAN::FrameDLC(dlc_ISR_raw);

            /* Convert from dlc to data length in bytes */
            std::uint8_t payloadLength_ISR = InterfaceGroup::FrameType::dlcToLength(dlc_ISR);

            /* Get the id */
            std::uint32_t id_ISR = (fc_->RAMn[mb_index * MB_Size_Words + 1]) & CAN_WMBn_ID_ID_MASK;

            /* Perform the harvesting of the payload, leveraging from native 32-bit transfers and since the FlexCAN
             * expects the data to be in big-endian order, a byte swap is required from the little-endian
             * transmission UAVCAN requirement */
            for (std::uint8_t i = 0;
                 i < (payloadLength_ISR >> 2) + std::min(1, static_cast<std::uint8_t>(payloadLength_ISR) & 0x3);
                 i++)
            {
                REV_BYTES_32(fc_->RAMn[mb_index * MB_Size_Words + MB_Data_Offset + i], data_ISR_word_[i]);
            }

            /* Harvest the frame's 16-bit hardware timestamp */
            std::uint64_t MB_timestamp = fc_->RAMn[mb_index * MB_Size_Words] & 0xFFFF;

            /* Instantiate monotonic object form a resolved timestamp */
            time::Monotonic timestamp_ISR = resolve_Timestamp(MB_timestamp);

            /* Create Frame object with constructor */
            CAN::Frame<CAN::TypeFD::MaxFrameSizeBytes> FrameISR(id_ISR,
                                                                reinterpret_cast<std::uint8_t*>(
                                                                    const_cast<std::uint32_t*>(data_ISR_word_)),
                                                                dlc_ISR,
                                                                timestamp_ISR);

            /* Insert the frame into the queue */
            frame_ISRbuffer_.push_back(FrameISR);
        }
        else
        {
            /* Increment the number of discarded frames due to full RX dequeue */
            discarded_frames_count_++;
        }

        /* Clear MB interrupt flag (write 1 to clear)*/
        fc_->IFLAG1 |= (1u << mb_index);
    }

    bool is_ready(bool ignore_write_available)
    {
        /* Poll for available frames in RX FIFO */
        if (!frame_ISRbuffer_.empty())
        {
            return true;
        }

        /* Check for available message buffers for transmission if ignore_write_available is false */
        else if (!ignore_write_available)
        {
            /* Poll the Inactive Message Buffer and Valid Priority Status flags for TX availability */
            if ((fc_->ESR2 & CAN_ESR2_IMB_MASK) && (fc_->ESR2 & CAN_ESR2_VPS_MASK))
            {
                return true;
            }
        }
        return false;
    }

    Result reconfigureFilters(const typename InterfaceManager::InterfaceGroupType::FrameType::Filter* filter_config,
                              std::size_t filter_config_length)
    {
        /* Input validation */
        if (filter_config_length > Filter_Count)
        {
            return Result::BadArgument;
        }

        enter_freeze_mode();

        /* Reset all previous filter configurations */
        for (std::size_t j = 0; j < CAN_RAMn_COUNT; ++j)
        {
            fc_->RAMn[j] = 0;
        }

        /* Clear the reception masks before configuring the new ones needed */
        for (std::size_t j = 0; j < CAN_RXIMR_COUNT; ++j)
        {
            fc_->RXIMR[j] = 0;
        }

        for (std::size_t j = 0; j < filter_config_length; ++j)
        {
            /* Setup reception MB's mask from input argument */
            fc_->RXIMR[j + 2] = filter_config[j].mask;

            /* Setup word 0 (4 Bytes) for ith MB
             * Extended Data Length      (EDL) = 1
             * Bit Rate Switch           (BRS) = 1
             * Error State Indicator     (ESI) = 0
             * Message Buffer Code      (CODE) = 4 ( Active for reception and empty )
             * Substitute Remote Request (SRR) = 0
             * ID Extended Bit           (IDE) = 1
             * Remote Tx Request         (RTR) = 0
             * Data Length Code          (DLC) = 0 ( Valid for transmission only )
             * Counter Time Stamp (TIME STAMP) = 0 ( Handled by hardware )
             */
            fc_->RAMn[(j + 2) * MB_Size_Words] = CAN_RAMn_DATA_BYTE_0(0xC4) | CAN_RAMn_DATA_BYTE_1(0x20);

            /* Setup Message buffers 2-7 29-bit extended ID from parameter */
            fc_->RAMn[(j + 2) * MB_Size_Words + 1] = filter_config[j].id;
        }

        exit_freeze_mode();

        return Result::Success;
    }

    Result read(InterfaceGroup::FrameType (&out_frames)[InterfaceGroup::RxFramesLen], std::size_t& out_frames_read)
    {
        /* Initialize return value and out_frames_read output reference value */
        Result status   = Result::SuccessNothing;
        out_frames_read = 0;

        /* Check if the ISR buffer isn't empty */
        if (!frame_ISRbuffer_.empty())
        {
            static_assert(InterfaceGroup::RxFramesLen == 1,
                          "We did not implement reading more than one message at a time.");

            /* Get the front element of the queue buffer */
            out_frames[0] = frame_ISRbuffer_.front();

            /* Pop the front element of the queue buffer */
            frame_ISRbuffer_.pop_front();

            /* Default RX number of frames read at once by this implementation is 1 */
            out_frames_read = InterfaceGroup::RxFramesLen;

            /* If read is successful, status is success */
            status = Result::Success;
        }

        /* Return status code */
        return status;
    }

    Result write(const InterfaceGroup::FrameType (&frames)[InterfaceGroup::TxFramesLen],
                 std::size_t  frames_len,
                 std::size_t& out_frames_written)
    {
        /* Initialize return value status */
        Result Status = Result::BufferFull;

        /* Input validation */
        if (frames_len > InterfaceGroup::TxFramesLen)
        {
            Status = Result::BadArgument;
        }

        /* Poll the Inactive Message Buffer and Valid Priority Status flags before checking for free MB's */
        if ((fc_->ESR2 & CAN_ESR2_IMB_MASK) && (fc_->ESR2 & CAN_ESR2_VPS_MASK))
        {
            /* Look for the lowest number free MB */
            std::uint8_t mb_index = (fc_->ESR2 & CAN_ESR2_LPTM_MASK) >> CAN_ESR2_LPTM_SHIFT;

            static_assert(InterfaceGroup::TxFramesLen == 1,
                          "We did not implement writing more than one message at a time.");

            /* Proceed with the tranmission */
            Status = messageBuffer_Transmit(mb_index, frames[0]);

            /* Argument assignment to 1 Frame transmitted successfully */
            out_frames_written = isSuccess(Status) ? 1 : 0;
        }

        /* Return status code */
        return Status;
    }

    std::uint32_t get_rx_overflows() const
    {
        return discarded_frames_count_;
    }

private:
    /**
     * See section 53.1.8.1 of the reference manual.
     * Idempotent helper method for entering freeze mode.
     */
    void enter_freeze_mode()
    {
        if (fc_->MCR & CAN_MCR_FRZACK_MASK)
        {
            // already in freeze mode
            return;
        }

        fc_->MCR &=
            ~CAN_MCR_MDIS_MASK; /* Unset disable bit (per procedure in section 53.1.8 of the reference manual) */
        fc_->MCR |= (CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK); /* Request freeze mode entry */

        /* Block for freeze mode entry */
        while (!(fc_->MCR & CAN_MCR_FRZACK_MASK))
        {
        };
        // TODO: implement the full freeze mode specification detailed by 53.1.8 of the reference manual including
        //       timeout with soft-reset.
    }

    void exit_freeze_mode()
    {
        /* Exit from freeze mode */
        fc_->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);
    }

    /*
     * Helper function for resolving the timestamp of a received frame from FlexCAN'S 16-bit overflowing timer.
     * Based on Pyuavcan's SourceTimeResolver class from which the terms source and target are used. Note: A maximum
     * of 820 microseconds is allowed for the reception ISR to reach this function starting from a successful frame
     * reception. The computation relies in that no more than a full period from the 16-bit timestamping timer
     * running at 80Mhz have passed, this could occur in deadlocks or priority inversion scenarios since 820 uSecs
     * constitute a significant amount of cycles, if this happens, timestamps would stop being monotonic. param
     * frame_timestamp Source clock read from the FlexCAN's peripheral timer. param  instance        The interface
     * instance number used by the ISR return time::Monotonic 64-bit timestamp resolved from 16-bit Flexcan's timer
     * samples.
     */
    time::Monotonic resolve_Timestamp(std::uint64_t frame_timestamp)
    {
        // TODO: can we enable quanta-level accuracy here by chaining the timer?
        /* Harvest the peripheral's current timestamp, this is the 16-bit overflowing source clock */
        std::uint64_t FlexCAN_timestamp = fc_->TIMER;

        /* Get an non-overflowing 64-bit timestamp, this is the target clock source */
        std::uint64_t target_source = static_cast<std::uint64_t>(
            (static_cast<std::uint64_t>(0xFFFFFFFF - LPIT0->TMR[1].CVAL) << 32) | (0xFFFFFFFF - LPIT0->TMR[0].CVAL));

        /* Compute the delta of time that occurred in the source clock */
        std::uint64_t source_delta = FlexCAN_timestamp > frame_timestamp ? FlexCAN_timestamp - frame_timestamp
                                                                         : frame_timestamp - FlexCAN_timestamp;

        /* Resolve the received frame's absolute timestamp and divide by 80 due the 80Mhz clock source
         * of both the source and target timers for converting them into the desired microseconds resolution */
        std::uint64_t resolved_timestamp_ISR = (target_source - source_delta) / 80;

        /* Instantiate the required Monotonic object from the resolved timestamp */
        return time::Monotonic::fromMicrosecond(resolved_timestamp_ISR);
    }

    /** @fn
     * Helper function for an immediate transmission through an available message buffer
     *
     * @param [in]  TX_MB_index  The index from an already polled available message buffer.
     * @param [in]  frame        The individual frame being transmitted.
     * @return libuavcan::Result:Success after a successful transmission request.
     */
    Result messageBuffer_Transmit(std::uint8_t TX_MB_index, const InterfaceGroup::FrameType& frame) const
    {
        /* Get data length of the frame wanted to be transmitted */
        std::uint_fast8_t payloadLength = frame.getDataLength();

        /* Get the frame's dlc */
        const std::uint32_t dlc =
            static_cast<std::underlying_type<libuavcan::media::CAN::FrameDLC>::type>(frame.getDLC());

        static_assert(InterfaceGroup::FrameType::MTUBytes % 4 == 0,
                      "We use optimizations that assume 4-word access to the frame buffer.");

        /* Casting from uint8 to native uint32 for faster payload transfer to transmission message buffer */
        const std::uint32_t* native_FrameData = reinterpret_cast<const std::uint32_t*>(frame.data);

        /* Fill up the payload's bytes, including the ones that don't add up to a full word e.g. 1,2,3,5,6,7 byte
         * data length payloads */
        for (std::uint8_t i = 0;
             i < (payloadLength >> 2) + std::min(1, (static_cast<std::uint8_t>(payloadLength) & 0x3));
             i++)
        {
            /* FlexCAN natively transmits the bytes in big-endian order, in order to transmit little-endian for
             * UAVCAN, a byte swap is required */
            REV_BYTES_32(native_FrameData[i], fc_->RAMn[TX_MB_index * MB_Size_Words + MB_Data_Offset + i]);
        }

        /* Fill up frame ID */
        fc_->RAMn[TX_MB_index * MB_Size_Words + 1] = frame.id & CAN_WMBn_ID_ID_MASK;

        /* Fill up word 0 of frame and transmit it
         * Extended Data Length       (EDL) = 1
         * Bit Rate Switch            (BRS) = 1
         * Error State Indicator      (ESI) = 0
         * Message Buffer Code       (CODE) = 12 ( Transmit data frame )
         * Substitute Remote Request  (SRR) = 0
         * ID Extended Bit            (IDE) = 1
         * Remote Tx Request          (RTR) = 0
         * Data Length Code           (DLC) = frame's dlc
         * Counter Time Stamp  (TIME STAMP) = 0 ( Handled by hardware )
         */
        fc_->RAMn[TX_MB_index * MB_Size_Words] =
            CAN_RAMn_DATA_BYTE_1(dlc) | CAN_WMBn_CS_DLC(dlc) | CAN_RAMn_DATA_BYTE_0(0xCC);

        /* Return successful transmission request status */
        return Result::Success;
    }

    /* Index in the FlexCAN array for this peripheral. */
    const unsigned index_;

    /* Pointer into the FlexCAN array for this peripheral. */
    CAN_Type* const fc_;

    /* Intermediate array for harvesting the received frame's payload in the ISR */
    volatile std::uint32_t data_ISR_word_[InterfaceGroup::FrameType::MTUBytes / 4u];

    /* Counter for the number of discarded messages due to the RX FIFO being full */
    volatile std::uint32_t discarded_frames_count_;

    /* Frame's reception FIFO as a dequeue with libuavcan's static memory pool, one for each available interface */
    std::deque<InterfaceGroup::FrameType,
               platform::memory::PoolAllocator<Frame_Capacity, sizeof(InterfaceGroup::FrameType)>>
        frame_ISRbuffer_;
};

// +--------------------------------------------------------------------------+
// | S32KInterfaceGroupImpl
// +--------------------------------------------------------------------------+

/**
 * Concrete type held internally and returned to the system via
 * libuavcan::media::S32K::InterfaceManager::startInterfaceGroup
 */
template <std::size_t InterfaceCount>
class S32KInterfaceGroupImpl : public InterfaceGroup
{
public:
    static_assert(InterfaceCount > 0, "Must have at least one CAN interface to define this type.");

    S32KInterfaceGroupImpl()
        : peripheral_storage_{}
    {}

    Result start(const typename InterfaceManager::InterfaceGroupType::FrameType::Filter* filter_config,
                 std::size_t                                                             filter_config_length)
    {
        /* CAN frames timestamping 64-bit timer initialization using chained LPIT channel 0 and 1 */

        /* Clock source option 6: (SPLLDIV2) at 80Mhz */
        PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_PCS(6);
        PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC(1); /* Clock gating to LPIT module */

        /* Enable module */
        LPIT0->MCR |= LPIT_MCR_M_CEN(1);

        /* Select 32-bit periodic Timer for both chained channels and timeouts timer (default)  */
        LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_MODE(0);
        LPIT0->TMR[1].TCTRL |= LPIT_TMR_TCTRL_MODE(0);
        LPIT0->TMR[2].TCTRL |= LPIT_TMR_TCTRL_MODE(0);

        /* Select chain mode for channel 1, this becomes the most significant 32 bits */
        LPIT0->TMR[1].TCTRL |= LPIT_TMR_TCTRL_CHAIN(1);

        /* Setup max reload value for both channels 0xFFFFFFFF */
        LPIT0->TMR[0].TVAL = LPIT_TMR_TVAL_TMR_VAL_MASK;
        LPIT0->TMR[1].TVAL = LPIT_TMR_TVAL_TMR_VAL_MASK;

        /* Start the timers */
        LPIT0->SETTEN |= LPIT_SETTEN_SET_T_EN_0(1) | LPIT_SETTEN_SET_T_EN_1(1);

        // TODO: check the datasheet on this one
        /* Verify that the least significant 32-bit timer is counting (not locked at 0xFFFFFFFF) */
        while (LPIT0->TMR[0].CVAL == LPIT_TMR_CVAL_TMR_CUR_VAL_MASK)
        {
        };

        bool did_one_succeed = false;
        bool did_any_fail    = false;
        /* FlexCAN instances initialization */
        for (std::size_t i = 0; i < InterfaceCount; ++i)
        {
            S32KFlexCan* interface = new (&peripheral_storage_[i]) S32KFlexCan(i);
            if (isSuccess(interface->start(filter_config, filter_config_length)))
            {
                did_one_succeed = true;
            }
            else
            {
                did_any_fail = true;
            }
        }

        /* Clock gating and multiplexing for the pins used */
        PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK; /* Clock gating to PORT E */
        PORTE->PCR[4] |= PORT_PCR_MUX(5);                /* CAN0_RX at PORT E pin 4 */
        PORTE->PCR[5] |= PORT_PCR_MUX(5);                /* CAN0_TX at PORT E pin 5 */

#if defined(MCU_S32K146) || defined(MCU_S32K148)

        PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK; /* Clock gating to PORT A */
        PORTA->PCR[12] |= PORT_PCR_MUX(3);               /* CAN1_RX at PORT A pin 12 */
        PORTA->PCR[13] |= PORT_PCR_MUX(3);               /* CAN1_TX at PORT A pin 13 */

        /* Set to LOW the standby (STB) pin in both transceivers of the UCANS32K146 node board */
        if (UAVCAN_NODE_BOARD_USED)
        {
            PORTE->PCR[11] |= PORT_PCR_MUX(1); /* MUX to GPIO */
            PTE->PDDR |= 1 << 11;              /* Set direction as output */
            PTE->PCOR |= 1 << 11;              /* Set the pin LOW */

            PORTE->PCR[10] |= PORT_PCR_MUX(1); /* Same as above */
            PTE->PDDR |= 1 << 10;
            PTE->PCOR |= 1 << 10;
        }

#endif

#if defined(MCU_S32K148)
        PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK; /* Clock gating to PORT B */
        PORTB->PCR[12] |= PORT_PCR_MUX(4);               /* CAN2_RX at PORT B pin 12 */
        PORTB->PCR[13] |= PORT_PCR_MUX(4);               /* CAN2_TX at PORT B pin 13 */
#endif
        if (did_any_fail)
        {
            return (did_one_succeed) ? Result::SuccessPartial : Result::Failure;
        }
        else
        {
            return Result::Success;
        }
    }

    virtual ~S32KInterfaceGroupImpl()
    {
        /* FlexCAN module deinitialization */
        for (std::size_t i = 0; i < InterfaceCount; ++i)
        {
            reinterpret_cast<S32KFlexCan*>(&peripheral_storage_[i])->~S32KFlexCan();
        }
    }

    virtual std::uint_fast8_t getInterfaceCount() const override
    {
        return TARGET_S32K_CANFD_COUNT;
    }

    virtual Result write(std::uint_fast8_t interface_index,
                         const FrameType (&frames)[TxFramesLen],
                         std::size_t  frames_len,
                         std::size_t& out_frames_written) override
    {
        /* Input validation */
        if (interface_index > InterfaceCount)
        {
            return Result::BadArgument;
        }
        else
        {
            return get_interface(interface_index - 1).write(frames, frames_len, out_frames_written);
        }
    }

    virtual Result read(std::uint_fast8_t interface_index,
                        FrameType (&out_frames)[RxFramesLen],
                        std::size_t& out_frames_read) override
    {
        out_frames_read = 0;

        /* Input validation */
        if (interface_index > InterfaceCount)
        {
            return Result::BadArgument;
        }
        else
        {
            return get_interface(interface_index - 1).read(out_frames, out_frames_read);
        }
    }

    virtual Result reconfigureFilters(const typename FrameType::Filter* filter_config,
                                      std::size_t                       filter_config_length) override
    {
        Result result = Result::Success;

        for (std::size_t i = 0; i < InterfaceCount; ++i)
        {
            result = get_interface(i).reconfigureFilters(filter_config, filter_config_length);
            if (isFailure(result))
            {
                return result;
            }
        }
        return result;
    }

    virtual Result select(duration::Monotonic timeout, bool ignore_write_available) override
    {
        /* Obtain timeout from object */
        std::uint32_t cycles_timeout = static_cast<std::uint32_t>(timeout.toMicrosecond());

        /* Initialization of delta variable for comparison */
        volatile std::uint32_t delta = 0;

        /* Disable LPIT channel 3 for loading */
        LPIT0->CLRTEN |= LPIT_CLRTEN_CLR_T_EN_3(1);

        /* Load LPIT with its maximum value */
        LPIT0->TMR[3].TVAL = LPIT_TMR_CVAL_TMR_CUR_VAL_MASK;

        /* Enable LPIT channel 3 for timeout start */
        LPIT0->SETTEN |= LPIT_SETTEN_SET_T_EN_3(1);

        /* Start of timed block */
        while (delta < cycles_timeout)
        {
            /* Poll in each of the available interfaces */
            for (std::size_t i = 0; i < InterfaceCount; ++i)
            {
                /* Poll for available frames in RX FIFO */
                if (get_interface(i).is_ready(ignore_write_available))
                {
                    return Result::Success;
                }
            }

            /* Get current value of delta */
            // TODO: verify the time units here.
            delta = LPIT_TMR_CVAL_TMR_CUR_VAL_MASK - (LPIT0->TMR[3].CVAL);
        }

        /* If this section is reached, means timeout occurred and return timeout status */
        return Result::SuccessTimeout;
    }

    /*
     * FlexCAN ISR for frame reception, implements a walkaround to the S32K1 FlexCAN's lack of a RX FIFO neither a
     * DMA triggering mechanism for CAN-FD frames in hardware. Completes in at max 7472 cycles when compiled with
     * g++ at -O3 param instance The FlexCAN peripheral instance number in which the ISR will be executed, starts at
     * 0. differing form this library's interface indexes that start at 1.
     */
    void isrHandler(std::uint8_t instance)
    {
        get_interface(instance).isrHandler();
    }

    std::uint32_t get_rx_overflows() const
    {
        std::uint32_t discarded_frames_count = 0;
        for (std::size_t i = 0; i < InterfaceCount; ++i)
        {
            discarded_frames_count += get_interface(i).get_rx_overflows();
        }
        return discarded_frames_count;
    }

private:
    S32KFlexCan& get_interface(std::size_t index)
    {
        return *reinterpret_cast<S32KFlexCan*>(&peripheral_storage_[index]);
    }

    typename std::aligned_storage<sizeof(S32KFlexCan), alignof(S32KFlexCan)>::type peripheral_storage_[InterfaceCount];
};

/* aligned storage allocated statically to store our single InterfaceGroup for this system. */
std::aligned_storage<sizeof(S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>),
                     alignof(S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>)>::type _group_storage;

S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* _group_singleton = nullptr;

}  // end anonymous namespace

// +--------------------------------------------------------------------------+
// | InterfaceManager
// +--------------------------------------------------------------------------+

Result InterfaceManager::startInterfaceGroup(const typename InterfaceGroupType::FrameType::Filter* filter_config,
                                             std::size_t                                           filter_config_length,
                                             InterfaceGroupPtrType&                                out_group)
{
    /* Initialize return values */
    out_group = nullptr;

    /* Input validation */
    if (filter_config_length > Filter_Count)
    {
        return Result::BadArgument;
    }

    S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* singleton_group = _group_singleton;

    if (singleton_group != nullptr)
    {
        // Called twice or called before stopInterfaceGroup.
        return Result::Failure;
    }

    /* If function ended successfully, return address of object member of type InterfaceGroup */
    S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* initialized_group =
        new (&_group_storage) S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>();
    Result status = initialized_group->start(filter_config, filter_config_length);

    _group_singleton = initialized_group;
    out_group        = initialized_group;

    /* Return code for start of InterfaceGroup */
    return status;
}

Result InterfaceManager::stopInterfaceGroup(InterfaceGroupPtrType& inout_group)
{
    (void) inout_group;
    return Result::NotImplemented;
}

std::size_t InterfaceManager::getMaxFrameFilters() const
{
    return Filter_Count;
}

}  // namespace S32K
}  // namespace media
}  // namespace libuavcan

extern "C"
{
    /*
     * Interrupt service routines handled by hardware in each frame reception, they are installed by the linker
     * in function of the number of instances available in the target MCU, the names match the ones from the defined
     * interrupt vector table from the startup code located in the startup_S32K14x.S file.
     */
    void CAN0_ORed_0_15_MB_IRQHandler()
    {
        libuavcan::media::S32K::S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* singleton_group =
            libuavcan::media::S32K::_group_singleton;
        if (singleton_group != nullptr)
        {
            singleton_group->isrHandler(0u);
        }
    }

#if defined(MCU_S32K146) || defined(MCU_S32K148)
    /* Interrupt for the 1st FlexCAN instance if available */
    void CAN1_ORed_0_15_MB_IRQHandler()
    {
        libuavcan::media::S32K::S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* singleton_group =
            libuavcan::media::S32K::_group_singleton;
        if (singleton_group != nullptr)
        {
            singleton_group->isrHandler(1u);
        }
    }
#endif

#if defined(MCU_S32K148)
    /* Interrupt for the 2nd FlexCAN instance if available */
    void CAN2_ORed_0_15_MB_IRQHandler()
    {
        libuavcan::media::S32K::S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* singleton_group =
            libuavcan::media::S32K::_group_singleton;
        if (singleton_group != nullptr)
        {
            singleton_group->isrHandler(2u);
        }
    }
#endif
}

#if defined(__GNUC__)
#    pragma GCC diagnostic pop
#endif
