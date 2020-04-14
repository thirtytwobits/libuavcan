/* Demo of libuavcan v1 media driver layer for the NXP S32K14x family
 * of aumototive-grade MCU's, running CANFD at 4Mbit/s in data phase
 * based on work by Abraham Rodriguez <abraham.rodriguez@nxp.com>
 *
 * Description:
 * Two rddrone_uavcan boards exchange messages with each other.
 */

/* Include media layer driver for NXP S32K MCU */
#include "libuavcan/media/S32K/canfd.hpp"
#include "device_registers.h"
#include "clocks_and_modes.h"
#include "LPUART.h"

#if defined(__GNUC__)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wold-style-cast"
#endif

extern "C"
{
    char data = 0;
    void PORT_init(void)
    {
        /*!
         * Pins definitions
         * ===================================================
         *
         * Pin number        | Function
         * ----------------- |------------------
         * PTC6              | UART1 TX
         * PTC7              | UART1 RX
         */
        PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTC        */
        PORTC->PCR[6] |= PORT_PCR_MUX(2);                /* Port C6: MUX = ALT2, UART1 TX */
        PORTC->PCR[7] |= PORT_PCR_MUX(2);                /* Port C7: MUX = ALT2, UART1 RX */
    }

    void WDOG_disable(void)
    {
        WDOG->CNT   = 0xD928C520; /* Unlock watchdog         */
        WDOG->TOVAL = 0x0000FFFF; /* Maximum timeout value   */
        WDOG->CS    = 0x00002100; /* Disable watchdog        */
    }

}  // extern "C"

void greenLED_init(void)
{
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTD */
    PORTD->PCR[16] = PORT_PCR_MUX(1);                /* Port D16: MUX = GPIO              */
    PTD->PDDR |= 1 << 16;                            /* Port D16: Data direction = output  */
}

int main()
{
    WDOG_disable();        /* Disable WDOG */
    SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal */
    SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
    NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
    PORT_init();           /* Configure ports */

    LPUART1_init();                                            /* Initialize LPUART @ 9600*/
    LPUART1_transmit_string("Running LPUART example\n\r");     /* Transmit char string */
    LPUART1_transmit_string("Input character to echo...\n\r"); /* Transmit char string */

#if !defined(LIBUAVCAN_TEST_NODE_ID)
    /* ID for the current UAVCAN node */
    constexpr std::uint32_t Node_ID = 1u;

#else
    /* ID and for the current UAVCAN node */
    constexpr std::uint32_t Node_ID = LIBUAVCAN_TEST_NODE_ID;

#endif

    constexpr std::uint32_t Node_Mask          = 0xF0; /* All care bits mask for frame filtering */
    constexpr std::uint32_t Node_message_shift = 4u;
    constexpr std::size_t   Node_Filters_Count = 1u; /* Number of ID's that the node will filter in */
    constexpr std::size_t   Node_Frame_Count   = 1u; /* Frames transmitted each time */
    constexpr std::size_t   First_Instance     = 1u; /* Interface instance used in this demo */
    constexpr std::uint32_t TestMessageId      = Node_ID | (Node_Mask & (1 << Node_message_shift));

    /* Size of the payload in bytes of the frame to be transmitted */
    constexpr std::uint16_t payload_length = libuavcan::media::S32K::InterfaceGroup::FrameType::MTUBytes;
    static_assert(payload_length % 4 == 0, "we're lazy and only handle 4-byte aligned MTU transports for this demo.");

    /* Frame's Data Length Code in function of it's payload length in bytes */
    libuavcan::media::CAN::FrameDLC demo_DLC =
        libuavcan::media::S32K::InterfaceGroup::FrameType::lengthToDlc(payload_length);

    /* 64-byte payload that will be exchanged between the nodes */
    std::uint32_t demo_payload[] = {0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD,
                                    0xAABBCCDD};

    static_assert(sizeof(demo_payload) == 16 * 4, "demo_payload is supposed to be 64-bytes.");

    /* Instantiate factory object */
    libuavcan::media::S32K::InterfaceManager demo_Manager;

    /* Create pointer to Interface object */
    libuavcan::media::S32K::InterfaceManager::InterfaceGroupPtrType demo_InterfacePtr;

    /* Create a frame that will reach NODE_B ID */
    libuavcan::media::S32K::InterfaceGroup::FrameType bouncing_frame_obj(TestMessageId,
                                                                         reinterpret_cast<std::uint8_t*>(demo_payload),
                                                                         demo_DLC);

    /* Array of frames to transmit (current implementation supports 1) */
    libuavcan::media::S32K::InterfaceGroup::FrameType bouncing_frame[Node_Frame_Count] = {bouncing_frame_obj};

    /* Instantiate the filter object that the current node will apply to receiving frames */
    libuavcan::media::S32K::InterfaceGroup::FrameType::Filter demo_Filter(TestMessageId, Node_Mask);

    std::uint32_t rx_msg_count = 0;

    /* Status variable for sequence control */
    libuavcan::Result status;

    /* Initialize the node with the previously defined filtering using factory method */
    status = demo_Manager.startInterfaceGroup(&demo_Filter, Node_Filters_Count, demo_InterfacePtr);

    greenLED_init();

    std::size_t frames_wrote = 0;
    if (libuavcan::isSuccess(status))
    {
        demo_InterfacePtr->write(First_Instance, bouncing_frame, Node_Frame_Count, frames_wrote);
    }

    /* Loop for retransmission of the frame */
    for (;;)
    {
        std::size_t frames_read = 0;

        if (libuavcan::isSuccess(status))
        {
            status = demo_InterfacePtr->read(First_Instance, bouncing_frame, frames_read);
        }

        if (frames_read)
        {
            /* Increment receive msg counter */
            rx_msg_count++;

            if (rx_msg_count == 1000)
            {
                PTD->PTOR |= 1 << 16; /* toggle output port D16 (Green LED) */
                rx_msg_count = 0;     /* and reset message counter */
            }

            /* Transmit back */
            std::size_t frames_wrote;

            /* Changed frame's ID for returning it back */
            bouncing_frame[0].id = TestMessageId;

            bouncing_frame[0].data[63] += 1;

            if (libuavcan::isSuccess(status))
            {
                status = demo_InterfacePtr->write(First_Instance, bouncing_frame, Node_Frame_Count, frames_wrote);
            }
        }
    }
}

#if defined(__GNUC__)
#    pragma GCC diagnostic pop
#endif
