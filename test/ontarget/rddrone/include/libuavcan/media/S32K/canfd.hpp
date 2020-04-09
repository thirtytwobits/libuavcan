/*
 * Copyright (c) 2020, NXP. All rights reserved.
 * Distributed under The MIT License.
 * Author: Abraham Rodriguez <abraham.rodriguez@nxp.com>
 */

/**
 * @file
 * Header driver file for the media layer of Libuavcan v1 targeting
 * the NXP S32K14 family of automotive grade MCU's, running
 * CAN-FD at 4Mbit/s data phase and 1Mbit/s in nominal phase.
 */

#ifndef CANFD_HPP_INCLUDED
#define CANFD_HPP_INCLUDED

#include "libuavcan/media/can.hpp"
#include "libuavcan/media/interfaces.hpp"

namespace libuavcan
{
namespace media
{
/**
 * @namespace S32K
 * Microcontroller-specific Interface classes, constants, variables and helper functions that make use
 * of the FlexCAN and LPIT peripherals for the current driver.
 */
namespace S32K
{
/**
 * @class
 * Implementation of the methods from libuavcan's media layer abstracct class InterfaceGroup,
 * with the template arguments listed below; for further details of this interface class,
 * refer to the template declaration in libuavcan/media/interface.hpp
 *
 * FrameT      = Frame with MTUBytesParam = MaxFrameSizeBytes (64 bytes for CAN-FD) and
 *                    FlagBitsCompareMask = 0x00 (default)
 * MaxTxFrames = 1 (default)
 * MaxRxFrames = 1 (default)
 */
class InterfaceGroup : public media::InterfaceGroup<media::CAN::Frame<media::CAN::TypeFD::MaxFrameSizeBytes>>
{
protected:
    /**
     * You can't instantiate or delete this object directly. Obtain references from
     * libuavcan::media::S32K::InterfaceGroup::startInterfaceGroup.
     */
    InterfaceGroup(){};
    virtual ~InterfaceGroup() = default;
public:
    // rule of six
    InterfaceGroup(const InterfaceGroup&) = delete;
    InterfaceGroup& operator=(const InterfaceGroup&) = delete;
    InterfaceGroup(const InterfaceGroup&&) = delete;
    InterfaceGroup& operator=(const InterfaceGroup&&) = delete;
};

/**
 * @class
 * Implementation of the methods from libuavcan's media layer abstracct class InterfaceManager,
 * with the template arguments listed below; for further details of this interface class,
 * refer to the template declaration in libuavcan/media/interface.hpp
 *
 * InterfaceGroupT    = S32K_InterfaceGroup  (previously declared class in the file)
 * InterfaceGroupPtrT = S32K_InterfaceGroup* (raw pointer)
 */
class InterfaceManager final : public media::InterfaceManager<InterfaceGroup, InterfaceGroup*>
{
public:
    /** @fn
     * Initialize the peripherals needed for the driver in the target MCU, also configures the
     * core clock sources to the Normal RUN profile,
     * @param [in]   filter_config         The filtering to apply equally to all FlexCAN instances.
     * @param [in]   filter_config_length  The length of the @p filter_config argument.
     * @param [out]  out_group             A pointer to set to the started group. This will be nullptr if the start
     * method fails.
     * @return libuavcan::Result::Success     if the group was successfully started and a valid pointer was returned.
     * @return libuavcan::Result::Failure     if the initialization fails at some point.
     *         The caller should assume that @p out_group is an invalid pointer if any failure is returned.
     * @return libuavcan::Result::BadArgument if filter_config_length is out of bound.
     */
    virtual Result startInterfaceGroup(const typename InterfaceGroupType::FrameType::Filter* filter_config,
                                       std::size_t                                           filter_config_length,
                                       InterfaceGroupPtrType&                                out_group) override;

    /** @fn
     * Release and deinitialize the peripherals needed for the current driver, disables all the FlexCAN
     * instances available, waiting for any pending transmission or reception to finish before. Also
     * resets the LPIT timer used for time-stamping, does not deconfigure the core and asynch clock sources.
    ¨* configured from startInterfaceGroup nor the pins.
     * @param [out]  inout_group Pointer that will be set to null
     * @return libuavcan::Result::Success. If the used peripherals were deinitialized properly.
     */
    virtual Result stopInterfaceGroup(InterfaceGroupPtrType& inout_group) override;

    /** @fn
     * Return the number of filters that the current UAVCAN node can support.
     * @return The maximum number of frame filters available for filter groups managed by this object,
     *         i.e. the number of combinations of ID and mask that each FlexCAN instance supports
     */
    virtual std::size_t getMaxFrameFilters() const override;
};

}  // END namespace S32K
}  // END namespace media
}  // END namespace libuavcan

#endif  // CANFD_HPP_INCLUDED
