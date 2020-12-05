/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Robert Wilbrandt
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Wilbrandt
   Desc:   Implementation of adapter class that enables usage of a
   GenericHWInterface in a combined_robot_hw
*/

#ifndef ROS_CONTROL_BOILERPLATE__COMBINABLE_GENERIC_HW_ADAPTER_IMPL_H
#define ROS_CONTROL_BOILERPLATE__COMBINABLE_GENERIC_HW_ADAPTER_IMPL_H

#include <ros_control_boilerplate/combinable_generic_hw_adapter.h>

namespace ros_control_boilerplate
{
template <typename ConcreteGenericHW>
CombinableGenericHWAdapter<ConcreteGenericHW>::CombinableGenericHWAdapter() : adapted_hw_interface_{ nullptr }
{
}

template <typename ConcreteGenericHW>
CombinableGenericHWAdapter<ConcreteGenericHW>::~CombinableGenericHWAdapter()
{
}

template <typename ConcreteGenericHW>
bool CombinableGenericHWAdapter<ConcreteGenericHW>::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  adapted_hw_interface_.reset(new ConcreteGenericHW(robot_hw_nh));
  adapted_hw_interface_->init();

  registerInterfaceManager(adapted_hw_interface_.get());

  return true;
}

template <typename ConcreteGenericHW>
void CombinableGenericHWAdapter<ConcreteGenericHW>::read(const ros::Time& time, const ros::Duration& period)
{
  adapted_hw_interface_->read(time, period);
}

template <typename ConcreteGenericHW>
void CombinableGenericHWAdapter<ConcreteGenericHW>::write(const ros::Time& time, const ros::Duration& period)
{
  adapted_hw_interface_->write(time, period);
}

template <typename ConcreteGenericHW>
bool CombinableGenericHWAdapter<ConcreteGenericHW>::checkForConflict(
    const std::list<hardware_interface::ControllerInfo>& info) const
{
  return adapted_hw_interface_->checkForConflict(info);
}

template <typename ConcreteGenericHW>
bool CombinableGenericHWAdapter<ConcreteGenericHW>::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  return adapted_hw_interface_->prepareSwitch(start_list, stop_list);
}

template <typename ConcreteGenericHW>
void CombinableGenericHWAdapter<ConcreteGenericHW>::doSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  adapted_hw_interface_->doSwitch(start_list, stop_list);
}

template <typename ConcreteGenericHW>
hardware_interface::RobotHW::SwitchState CombinableGenericHWAdapter<ConcreteGenericHW>::switchResult() const
{
  return adapted_hw_interface_->switchResult();
}

template <typename ConcreteGenericHW>
hardware_interface::RobotHW::SwitchState
CombinableGenericHWAdapter<ConcreteGenericHW>::switchResult(const hardware_interface::ControllerInfo& controller) const
{
  return adapted_hw_interface_->switchResult(controller);
}

}  // namespace ros_control_boilerplate

#endif
