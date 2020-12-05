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
   Desc:   Adapter class that enables usage of a GenericHWInterface in a
   combined_robot_hw
*/

#ifndef ROS_CONTROL_BOILERPLATE__COMBINABLE_GENERIC_HW_ADPAPTER_H
#define ROS_CONTROL_BOILERPLATE__COMBINABLE_GENERIC_HW_ADPAPTER_H

#include <memory>
#include <hardware_interface/robot_hw.h>
#include <ros_control_boilerplate/generic_hw_interface.h>

namespace ros_control_boilerplate
{
/**
 * \brief Adapter for GenericHWInterfaces in combined_robot_hw
 *
 * TODO: Justification, why do we need this -> non-matching interface
 * TODO: Usage, how to export custom GenericHWInterface
 *
 * \tparam ConcreteGenericHW Concrete GenericHWInterface to adapt
 */
template <typename ConcreteGenericHW>
class CombinableGenericHWAdapter : public hardware_interface::RobotHW
{
public:
  /** \brief Constructor */
  CombinableGenericHWAdapter();

  /** \brief Destructor */
  virtual ~CombinableGenericHWAdapter();

  /** \brief Creates and initializes the adapted GenericHWInterface
   *
   * \param root_nh Root level ROS node handle
   * \param robot_hw_nh ROS node handle for the robot namespace
   *
   * \returns Always returns true, as these steps cannot fail
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /** \brief Reads the state from the robot hardware
   *
   * \note This just delegates to the adapted GenericHWInterface
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Write commands to the robot hardware
   *
   * \note This just delegates to the adapted GenericHWInterface
   *
   * \param time Current time
   * \param period Duration of the current control loop iteration
   */
  virtual void write(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Check if given set of controllers is allowed to run simultaneously
   *
   * \note This just delegates to the adapted GenericHWInterface
   *
   * \param info Set of controllers to check
   */
  virtual bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const override;

  /** \brief Check if given controller switches are currently possible and prepare for them
   *
   * \note This just delegates to the adapted GenericHWInterface
   *
   * \param start_list Controllers that would get started, disjoint from stop_list
   * \param stop_list Controllers that would get stopped, disjoint from start_list
   *
   * \returns If this set of controller switches is currently possible
   */
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  /** \brief Perform all hardware-side changes to allow for controller switches
   *
   * \note This just delegates to the adapted GenericHWInterface
   *
   * \param start_list Controllers that would get started, disjoint from stop_list
   * \param stop_list Controllers that would get stopped, disjoint from start_list
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  /** \brief Return the state of the last doSwitch()
   *
   * \note This just delegates to the adapted GenericHWInterface
   *
   * \returns State of the last doSwitch()
   */
  virtual RobotHW::SwitchState switchResult() const override;

  /** \brief Return the state of the last doSwitch() for a given controller
   *
   * \note This just delegates to the adapted GenericHWInterface
   *
   * \param controller Controller to check state for
   *
   * \returns State of the last doSwitch() for controller
   */
  virtual RobotHW::SwitchState switchResult(const hardware_interface::ControllerInfo& controller) const override;

protected:
  std::shared_ptr<ConcreteGenericHW> adapted_hw_interface_;
};

}  // namespace ros_control_boilerplate

#include <ros_control_boilerplate/combinable_generic_hw_adapter_impl.hpp>

#endif
