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
/* Number of capable CAN-FD FlexCAN instances */
constexpr static std::uint_fast8_t CANFD_Count = TARGET_S32K_CANFD_COUNT;

/* Tunable frame capacity for the ISR reception FIFO, each frame adds 80 bytes of required .bss memory */
constexpr static std::size_t Frame_Capacity = 40u;

/* Number of filters supported by a single FlexCAN instance */
constexpr static std::uint8_t Filter_Count = 5u;

/* Lookup table for NVIC IRQ numbers for each FlexCAN instance */
constexpr static std::uint32_t FlexCAN_NVIC_Indices[][2u] = {{2u, 0x20000}, {2u, 0x1000000}, {2u, 0x80000000}};

/* Array of each FlexCAN instance's addresses for dereferencing from */
constexpr static CAN_Type* FlexCAN[] = CAN_BASE_PTRS;

/* Lookup table for FlexCAN indices in PCC register */
constexpr static std::uint8_t PCC_FlexCAN_Index[] = {36u, 37u, 43u};

/* Size in words (4 bytes) of the offset between the location of message buffers in FlexCAN's dedicated RAM */
constexpr static std::uint8_t MB_Size_Words = 18u;

/* Offset in words for reaching the payload of a message buffer */
constexpr static std::uint8_t MB_Data_Offset = 2u;

/* Number of cycles to wait for the timed polls, corresponding to a timeout of 1/(80Mhz) * 2^24 = 0.2 seconds approx */
constexpr static std::uint32_t cycles_timeout = 0xFFFFFF;

/*
 * Enumeration for converting from a bit number to an index, used for some registers where a bit flag for a nth
 * message buffer is represented as a bit left shifted nth times. e.g. 2nd MB is 0b100 = 4 = (1 << 2)
 */
enum MB_bit_to_index : std::uint8_t
{
    MessageBuffer0 = 0x1,  /* Number representing the bit for the zeroth MB (1 << 2) */
    MessageBuffer1 = 0x2,  /* Number for the bit of the first  MB (1 << 3) */
    MessageBuffer2 = 0x4,  /* Number for the bit of the second MB (1 << 2) */
    MessageBuffer3 = 0x8,  /* Number for the bit of the third  MB (1 << 3) */
    MessageBuffer4 = 0x10, /* Number for the bit of the fourth MB (1 << 4) */
    MessageBuffer5 = 0x20, /* Number for the bit of the fifth  MB (1 << 5) */
    MessageBuffer6 = 0x40, /* Number for the bit of the sixth  MB (1 << 6) */
};

/*
 * Helper function for block polling a bit flag until it is set with a timeout of 0.2 seconds using a LPIT timer,
 * the argument list and usage reassembles the classic block polling while loop, and instead of using a third
 * argument to decide if it'ss a timed block for a clear or set, the two flavors of the function are provided.
 *
 * param  flagRegister Register where the flag is located.
 * param  flagMask     Mask to AND'nd with the register for isolating the flag.
 * return Result::Success If the flag set before the timeout expiration..
 * return Result::Failure If a timeout ocurred before the desired flag set.
 */
Result flagPollTimeout_Set(volatile std::uint32_t& flagRegister, std::uint32_t flag_Mask)
{
    /* Initialization of delta for timeout measurement */
    volatile std::uint32_t delta = 0;

    /* Disable LPIT channel 2 for loading */
    LPIT0->CLRTEN |= LPIT_CLRTEN_CLR_T_EN_2(1);

    /* Load LPIT with its maximum value */
    LPIT0->TMR[2].TVAL = LPIT_TMR_CVAL_TMR_CUR_VAL_MASK;

    /* Enable LPIT channel 2 for timeout start */
    LPIT0->SETTEN |= LPIT_SETTEN_SET_T_EN_2(1);

    /* Start of timed block */
    while (delta < cycles_timeout)
    {
        /* Check if the flag has been set */
        if (flagRegister & flag_Mask)
        {
            return Result::Success;
        }

        /* Get current value of delta */
        delta = LPIT_TMR_CVAL_TMR_CUR_VAL_MASK - (LPIT0->TMR[2].CVAL);
    }

    /* If this section is reached, means timeout ocurred and return error status is returned */
    return Result::Failure;
}

/*
 * Helper function for block polling a bit flag until it is cleared with a timeout of 0.2 seconds using a LPIT timer
 *
 * param  flagRegister Register where the flag is located.
 * param  flagMask     Mask to AND'nd with the register for isolating the flag.
 * return Result::Success If the flag cleared before the timeout expiration..
 * return Result::Failure If a timeout ocurred before the desired flag cleared.
 */
Result flagPollTimeout_Clear(volatile std::uint32_t& flagRegister, std::uint32_t flag_Mask)
{
    /* Initialization of delta for timeout measurement */
    volatile std::uint32_t delta = 0;

    /* Disable LPIT channel 2 for loading */
    LPIT0->CLRTEN |= LPIT_CLRTEN_CLR_T_EN_2(1);

    /* Load LPIT with its maximum value */
    LPIT0->TMR[2].TVAL = LPIT_TMR_CVAL_TMR_CUR_VAL_MASK;

    /* Enable LPIT channel 2 for timeout start */
    LPIT0->SETTEN |= LPIT_SETTEN_SET_T_EN_2(1);

    /* Start of timed block */
    while (delta < cycles_timeout)
    {
        /* Check if the flag has been set */
        if (!(flagRegister & flag_Mask))
        {
            return Result::Success;
        }

        /* Get current value of delta */
        delta = LPIT_TMR_CVAL_TMR_CUR_VAL_MASK - (LPIT0->TMR[2].CVAL);
    }

    /* If this section is reached, means timeout ocurred and return error status is returned */
    return Result::Failure;
}

// +--------------------------------------------------------------------------+
// | S32KFlexCan
// +--------------------------------------------------------------------------+

/**
 * Per interface implementation.
 */
class S32KFlexCan final
{
public:
    S32KFlexCan(unsigned                                                                peripheral_index,
                const typename InterfaceManager::InterfaceGroupType::FrameType::Filter* filter_config,
                std::size_t                                                             filter_config_length)
        : index_(peripheral_index)
        , data_ISR_word_{0}
        , discarded_frames_count_(0)
        , frame_ISRbuffer_()
    {
        CAN_Type* fc = FlexCAN[index_];

        /* FlexCAN instance initialization */
        PCC->PCCn[PCC_FlexCAN_Index[index_]] = PCC_PCCn_CGC_MASK; /* FlexCAN clock gating */
        fc->MCR |= CAN_MCR_MDIS_MASK;                      /* Disable FlexCAN module for clock source selection */
        fc->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;               /* Clear any previous clock source configuration */
        fc->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;                /* Select SYS_CLK as source (80Mhz)*/
        fc->MCR &= ~CAN_MCR_MDIS_MASK;                     /* Enable FlexCAN peripheral */
        fc->MCR |= (CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK); /* Request freeze mode etry */

        /* Block for freeze mode entry */
        while (!(fc->MCR & CAN_MCR_FRZACK_MASK))
        {
        };

        /* Next configurations are only permitted in freeze mode */
        fc->MCR |= CAN_MCR_FDEN_MASK |          /* Habilitate CANFD feature */
                   CAN_MCR_FRZ_MASK;            /* Enable freeze mode entry when HALT bit is asserted */
        fc->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK; /* Activate the use of ISO 11898-1 CAN-FD standard */

        /* CAN Bit Timing (CBT) configuration for a nominal phase of 1 Mbit/s with 80 time quantas,
            in accordance with Bosch 2012 specification, sample point at 83.75% */
        fc->CBT |= CAN_CBT_BTF_MASK |     /* Enable extended bit timing configurations for CAN-FD
                                                                                                                    for
                                                        setting     up separetely nominal and data phase */
                   CAN_CBT_EPRESDIV(0) |  /* Prescaler divisor factor of 1 */
                   CAN_CBT_EPROPSEG(46) | /* Propagation segment of 47 time quantas */
                   CAN_CBT_EPSEG1(18) |   /* Phase buffer segment 1 of 19 time quantas */
                   CAN_CBT_EPSEG2(12) |   /* Phase buffer segment 2 of 13 time quantas */
                   CAN_CBT_ERJW(12);      /* Resynchronization jump width same as PSEG2 */

        /* CAN-FD Bit Timing (FDCBT) for a data phase of 4 Mbit/s with 20 time quantas,
            in accordance with Bosch 2012 specification, sample point at 75% */
        fc->FDCBT |= CAN_FDCBT_FPRESDIV(0) | /* Prescaler divisor factor of 1 */
                     CAN_FDCBT_FPROPSEG(7) | /* Propagation semgment of 7 time quantas
                                                                             (only register that doesn't add 1) */
                     CAN_FDCBT_FPSEG1(6) |   /* Phase buffer segment 1 of 7 time quantas */
                     CAN_FDCBT_FPSEG2(4) |   /* Phase buffer segment 2 of 5 time quantas */
                     CAN_FDCBT_FRJW(4);      /* Resynchorinzation jump width same as PSEG2 */

        /* Additional CAN-FD configurations */
        fc->FDCTRL |= CAN_FDCTRL_FDRATE_MASK | /* Enable bit rate switch in data phase of frame */
                      CAN_FDCTRL_TDCEN_MASK |  /* Enable transceiver delay compensation */
                      CAN_FDCTRL_TDCOFF(5) |   /* Setup 5 cycles for data phase sampling delay */
                      CAN_FDCTRL_MBDSR0(3);    /* Setup 64 bytes per message buffer (7 MB's) */

        /* Message buffers are located in a dedicated RAM inside FlexCAN, they aren't affected by reset,
         * so they must be explicitly initialized, they total 128 slots of 4 words each, which sum
         * to 512 bytes, each MB is 72 byte in size ( 64 payload and 8 for headers )
         */
        for (std::uint8_t j = 0; j < CAN_RAMn_COUNT; j++)
        {
            fc->RAMn[j] = 0;
        }

        /* Clear the reception masks before configuring the ones needed */
        for (std::uint8_t j = 0; j < CAN_RXIMR_COUNT; j++)
        {
            fc->RXIMR[j] = 0;
        }

        /* Setup maximum number of message buffers as 7, 0th and 1st for transmission and 2nd-6th for RX */
        fc->MCR &= ~CAN_MCR_MAXMB_MASK;                     /* Clear previous configuracion of MAXMB, default is 0xF */
        fc->MCR |= CAN_MCR_MAXMB(6) | CAN_MCR_SRXDIS_MASK | /* Disable self-reception of frames if ID matches */
                   CAN_MCR_IRMQ_MASK;                       /* Enable individual message buffer masking */

        /* Setup Message buffers 2nd-6th for reception and set filters */
        for (std::uint8_t j = 0; j < filter_config_length; j++)
        {
            /* Setup reception MB's mask from input argument */
            fc->RXIMR[j + 2] = filter_config[j].mask;

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
            fc->RAMn[(j + 2) * MB_Size_Words] = CAN_RAMn_DATA_BYTE_0(0xC4) | CAN_RAMn_DATA_BYTE_1(0x20);

            /* Setup Message buffers 2-7 29-bit extended ID from parameter */
            fc->RAMn[(j + 2) * MB_Size_Words + 1] = filter_config[j].id;
        }
    }

    ~S32KFlexCan()
    {
        /* Disable FlexCAN module */
        FlexCAN[index_]->MCR |= CAN_MCR_MDIS_MASK;

        /* Poll for Low Power ACK, waits for current transmission/reception to finish */
        while (FlexCAN[index_]->MCR, CAN_MCR_LPMACK_MASK)
            ;

        /* Disable FlexCAN clock gating */
        PCC->PCCn[PCC_FlexCAN_Index[index_]] &= ~PCC_PCCn_CGC_MASK;
    }

    S32KFlexCan(const S32KFlexCan&) = delete;
    S32KFlexCan& operator=(const S32KFlexCan&) = delete;
    S32KFlexCan(S32KFlexCan&&)    = delete;
    S32KFlexCan& operator=(S32KFlexCan&&) = delete;

    void isrHandler()
    {
        /* Initialize variable for finding which MB received */
        std::uint8_t mb_index = 0;

        /* Check which RX MB caused the interrupt (0b1111100) mask for 2nd-6th MB */
        switch (FlexCAN[index_]->IFLAG1 & 124)
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
        }

        /* Validate that the index didn't get stuck at 0, this would be invalid since MB's 0th and 1st are TX */
        if (mb_index)
        {
            /* Receive a frame only if the buffer its under its capacity */
            if (frame_ISRbuffer_.size() <= Frame_Capacity)
            {
                /* Harvest the Message buffer, read of the control and status word locks the MB */

                /* Get the raw DLC from the message buffer that received a frame */
                std::uint32_t dlc_ISR_raw =
                    ((FlexCAN[index_]->RAMn[mb_index * MB_Size_Words]) & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;

                /* Create CAN::FrameDLC type variable from the raw dlc */
                CAN::FrameDLC dlc_ISR = CAN::FrameDLC(dlc_ISR_raw);

                /* Convert from dlc to data length in bytes */
                std::uint8_t payloadLength_ISR = InterfaceGroup::FrameType::dlcToLength(dlc_ISR);

                /* Get the id */
                std::uint32_t id_ISR = (FlexCAN[index_]->RAMn[mb_index * MB_Size_Words + 1]) & CAN_WMBn_ID_ID_MASK;

                /* Perform the harvesting of the payload, leveraging from native 32-bit transfers and since the FlexCAN
                 * expects the data to be in big-endian order, a byte swap is required from the little-endian
                 * transmission UAVCAN requirement */
                for (std::uint8_t i = 0;
                     i < (payloadLength_ISR >> 2) + std::min(1, static_cast<std::uint8_t>(payloadLength_ISR) & 0x3);
                     i++)
                {
                    REV_BYTES_32(FlexCAN[index_]->RAMn[mb_index * MB_Size_Words + MB_Data_Offset + i],
                                 data_ISR_word_[i]);
                }

                /* Harvest the frame's 16-bit hardware timestamp */
                std::uint64_t MB_timestamp = FlexCAN[index_]->RAMn[mb_index * MB_Size_Words] & 0xFFFF;

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
            FlexCAN[index_]->IFLAG1 |= (1u << mb_index);
        }
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
            if ((FlexCAN[index_]->ESR2 & CAN_ESR2_IMB_MASK) && (FlexCAN[index_]->ESR2 & CAN_ESR2_VPS_MASK))
            {
                return true;
            }
        }
        return false;
    }

    Result reconfigureFilters(const typename InterfaceManager::InterfaceGroupType::FrameType::Filter* filter_config, std::size_t filter_config_length)
    {
        /* Initialize return value status */
        Result Status = Result::Success;

        /* Input validation */
        if (filter_config_length > Filter_Count)
        {
            Status = Result::BadArgument;
        }

        if (isSuccess(Status))
        {
            /* Enter freeze mode for filter reconfiguration */
            FlexCAN[i]->MCR |= (CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);

            /* Block for freeze mode entry, halts any transmission or reception */
            if (isSuccess(Status))
            {
                /* Reset all previous filter configurations */
                for (std::size_t j = 0; j < CAN_RAMn_COUNT; ++j)
                {
                    FlexCAN[index_]->RAMn[j] = 0;
                }

                /* Clear the reception masks before configuring the new ones needed */
                for (std::size_t j = 0; j < CAN_RXIMR_COUNT; ++j)
                {
                    FlexCAN[i]->RXIMR[j] = 0;
                }

                for (std::size_t j = 0; j < filter_config_length; ++j)
                {
                    /* Setup reception MB's mask from input argument */
                    FlexCAN[index_]->RXIMR[j + 2] = filter_config[j].mask;

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
                    FlexCAN[index_]->RAMn[(j + 2) * MB_Size_Words] =
                        CAN_RAMn_DATA_BYTE_0(0xC4) | CAN_RAMn_DATA_BYTE_1(0x20);

                    /* Setup Message buffers 2-7 29-bit extended ID from parameter */
                    FlexCAN[index_]->RAMn[(j + 2) * MB_Size_Words + 1] = filter_config[j].id;
                }

                /* Freeze mode exit request */
                FlexCAN[index_]->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);

                /* Block for freeze mode exit */
                if (isSuccess(Status))
                {
                    Status = flagPollTimeout_Clear(FlexCAN[index_]->MCR, CAN_MCR_FRZACK_MASK);

                    /* Block until module is ready */
                    if (isSuccess(Status))
                    {
                        Status = flagPollTimeout_Clear(FlexCAN[index_]->MCR, CAN_MCR_NOTRDY_MASK);
                    }
                }
            }
        }

        Result read(FrameType(&out_frames)[InterfaceGroup::RxFramesLen], std::size_t & out_frames_read)
        {
            /* Initialize return value and out_frames_read output reference value */
            Result status   = Result::SuccessNothing;
            out_frames_read = 0;

            /* Check if the ISR buffer isn't empty */
            if (!frame_ISRbuffer_.empty())
            {
                // TODO handle where RxFramesLen > 1

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

        Result write(const FrameType(&frames)[InterfaceGroup::TxFramesLen],
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
            if ((FlexCAN[index_]->ESR2 & CAN_ESR2_IMB_MASK) && (FlexCAN[index_]->ESR2 & CAN_ESR2_VPS_MASK))
            {
                /* Look for the lowest number free MB */
                std::uint8_t mb_index = (FlexCAN[index_]->ESR2 & CAN_ESR2_LPTM_MASK) >> CAN_ESR2_LPTM_SHIFT;

                /* Proceed with the tranmission */
                Status = messageBuffer_Transmit(mb_index, frames[0]);

                /* Argument assignment to 1 Frame transmitted successfully */
                out_frames_written = isSuccess(Status) ? TxFramesLen : 0;
            }

            /* Return status code */
            return Status;
        }

    private:
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
            /* Harvest the peripheral's current timestamp, this is the 16-bit overflowing source clock */
            std::uint64_t FlexCAN_timestamp = FlexCAN[index_]->TIMER;

            /* Get an non-overflowing 64-bit timestamp, this is the target clock source */
            std::uint64_t target_source =
                static_cast<std::uint64_t>((static_cast<std::uint64_t>(0xFFFFFFFF - LPIT0->TMR[1].CVAL) << 32) |
                                           (0xFFFFFFFF - LPIT0->TMR[0].CVAL));

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
        Result messageBuffer_Transmit(std::uint8_t TX_MB_index, const FrameType& frame) const
        {
            /* Get data length of the frame wanted to be transmitted */
            std::uint_fast8_t payloadLength = frame.getDataLength();

            /* Get the frame's dlc */
            const std::uint32_t dlc =
                static_cast<std::underlying_type<libuavcan::media::CAN::FrameDLC>::type>(frame.getDLC());

            /* Casting from uint8 to native uint32 for faster payload transfer to transmission message buffer */
            std::uint32_t* native_FrameData = reinterpret_cast<std::uint32_t*>(const_cast<std::uint8_t*>(frame.data));

            /* Fill up the payload's bytes, including the ones that don't add up to a full word e.g. 1,2,3,5,6,7 byte
             * data length payloads */
            for (std::uint8_t i = 0;
                 i < (payloadLength >> 2) + std::min(1, (static_cast<std::uint8_t>(payloadLength) & 0x3));
                 i++)
            {
                /* FlexCAN natively transmits the bytes in big-endian order, in order to transmit little-endian for
                 * UAVCAN, a byte swap is required */
                REV_BYTES_32(native_FrameData[i],
                             FlexCAN[iface_index]->RAMn[TX_MB_index * MB_Size_Words + MB_Data_Offset + i]);
            }

            /* Fill up frame ID */
            FlexCAN[index_]->RAMn[TX_MB_index * MB_Size_Words + 1] = frame.id & CAN_WMBn_ID_ID_MASK;

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
            FlexCAN[index_]->RAMn[TX_MB_index * MB_Size_Words] =
                CAN_RAMn_DATA_BYTE_1(dlc) | CAN_WMBn_CS_DLC(dlc) | CAN_RAMn_DATA_BYTE_0(0xCC);

            /* After a successful transmission the interrupt flag of the corresponding message buffer is set, poll with
             * timeout for it */
            Result Status = flagPollTimeout_Set(FlexCAN[index_]->IFLAG1, 1u << TX_MB_index);

            /* Clear the flag previously polled (W1C register) */
            FlexCAN[index_]->IFLAG1 |= 1u << TX_MB_index;

            /* Return successful transmission request status */
            return Status;
        }

        unsigned index_;

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
        S32KInterfaceGroupImpl(const typename InterfaceManager::InterfaceGroupType::FrameType::Filter* filter_config,
                               std::size_t filter_config_length)
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

            /* Verify that the least significant 32-bit timer is counting (not locked at 0xFFFFFFFF) */
            while (LPIT0->TMR[0].CVAL == LPIT_TMR_CVAL_TMR_CUR_VAL_MASK)
            {
            };

            /* FlexCAN instances initialization */
            for (std::size_t i = 0; i < InterfaceCount; ++i)
            {
                new (&peripheral_storage_[i]) S32KFlexCan(i, filter_config, filter_config_length);
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
        }

        virtual ~S32KInterfaceGroupImpl()
        {
            /* FlexCAN module deinitialization */
            for (std::size_t i = 0; i < InterfaceCount; ++i)
            {
                reinterpret_cast<S32KFlexCan*>(&peripheral_storage_[i])->~S32KFlexCan();
            }

            /* Reset LPIT timer peripheral, (resets all except the MCR register) */
            LPIT0->MCR |= LPIT_MCR_SW_RST(1);

            /* Verify that the timer did reset (locked at 0xFFFFFFFF) */
            while (LPIT0->TMR[0].CVAL != LPIT_TMR_CVAL_TMR_CUR_VAL_MASK)
            {
            }
            /* Clear the reset bit since it isn't cleared automatically */
            LPIT0->MCR &= ~LPIT_MCR_SW_RST_MASK;

            /* Disable the clock to the LPIT's timers */
            LPIT0->MCR &= ~LPIT_MCR_M_CEN_MASK;

            /* Disable LPIT clock gating */
            PCC->PCCn[PCC_LPIT_INDEX] &= ~PCC_PCCn_CGC_MASK;
        }

        virtual std::uint_fast8_t getInterfaceCount() const override
        {
            return CANFD_Count;
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
            for (std::size_t i = 0; i < InterfaceCount; ++i)
            {
                result = get_interface(i).reconfigureFilters(filter_config, filter_config_length);
                if (isFailure(result))
                {
                    return result;
                }
            }
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
            /* Perform the ISR atomically */
            DISABLE_INTERRUPTS()

            get_interface(instance).isrHandler();

            /* Enable interrupts back */
            ENABLE_INTERRUPTS()
        }

        std::uint32_t get_rx_overflows() const
        {
            return discarded_frames_count_;
        }

        S32KFlexCan& get_interface(std::size_t index)
        {
            return *reinterpret_cast<S32KFlexCan*>(&peripheral_storage_[index]);
        }

    private:
        typename std::aligned_storage<sizeof(S32KFlexCan), alignof(S32KFlexCan)>::type
            peripheral_storage_[InterfaceCount];
    };

    /* aligned storage allocated statically to store our single InterfaceGroup for this system. */
    std::aligned_storage<sizeof(S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>),
                         alignof(S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>)>::type _group_storage;

    S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* _group_singleton = nullptr;

}  // end anonymous namespace

// +--------------------------------------------------------------------------+
// | InterfaceManager
// +--------------------------------------------------------------------------+

Result
InterfaceManager::startInterfaceGroup(const typename InterfaceGroupType::FrameType::Filter* filter_config,
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

    // TODO : NVIC clear pending then disable CAN rx interrupts.

    S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* singleton_group = _group_singleton;

    if (singleton_group != nullptr)
    {
        // Called twice or called before stopInterfaceGroup.
        return Result::Failure;
    }

    /* If function ended successfully, return address of object member of type InterfaceGroup */
    S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* initialized_group =
        new (&_group_storage) S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>(filter_config, filter_config_length);

    _group_singleton = initialized_group;
    out_group        = initialized_group;

    /* Return code for start of InterfaceGroup */
    return Result::Success;
}

Result InterfaceManager::stopInterfaceGroup(InterfaceGroupPtrType& inout_group)
{
    // TODO : NVIC clear pending then disable CAN rx interrupts.

    S32KInterfaceGroupImpl<TARGET_S32K_CANFD_COUNT>* singleton_group = _group_singleton;

    if (singleton_group == nullptr)
    {
        // Called twice or before startInterfaceGroup
        return Result::Failure;
    }
    else if (inout_group != static_cast<InterfaceGroupPtrType>(singleton_group))
    {
        // Called with a group other than the one provided in startInterfaceGroup
        return Result::BadArgument;
    }

    _group_singleton = nullptr;
    inout_group      = nullptr;
    singleton_group->~S32KInterfaceGroupImpl();
    return Result::Success;
}

std::size_t InterfaceManager::getMaxFrameFilters() const
{
    return Filter_Count;
}

}  // namespace
}  // namespace S32K
}  // namespace media

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
