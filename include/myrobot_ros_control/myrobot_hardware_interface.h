/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface that performs a perfect control loop for simulation
*/

#ifndef MYROBOT_ROS_CONTROL__MYROBOT_HARDWARE_INTERFACE_H
#define MYROBOT_ROS_CONTROL__MYROBOT_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

namespace ros_control_boilerplate
{

// For simulation only - determines how fast a trajectory is followed
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

/// \brief Hardware interface for a robot
class MyRobotHardwareInterface: public hardware_interface::RobotHW
{
private:

  ros::NodeHandle                              nh_;

  // Interfaces
  hardware_interface::JointStateInterface      joint_state_interface_;
  hardware_interface::PositionJointInterface   position_joint_interface_;
  hardware_interface::VelocityJointInterface   velocity_joint_interface_;
  hardware_interface::EffortJointInterface     effort_joint_interface_;

  // Timing
  ros::Timer                                   non_realtime_loop_;
  ros::Duration                                control_period_;
  ros::Duration                                elapsed_time_;
  double                                       loop_hz_;

  // Shared memory
  std::vector<std::string>                     joint_names_;
  std::vector<double>                          joint_position_;
  std::vector<double>                          joint_velocity_;
  std::vector<double>                          joint_effort_;
  std::vector<double>                          joint_position_command_;
  std::vector<double>                          joint_velocity_command_;
  std::vector<double>                          joint_effort_command_;
  int                                          num_joints_;
  int                                          joint_mode_; // position, velocity, or effort

  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Simulated controller
  double                                       p_error_, v_error_, e_error_;

public:
  /// \brief Constructor.
  ///
  /// \param nh  Node handle for topics.
  MyRobotHardwareInterface(ros::NodeHandle& nh);

  /// \brief Destructor.
  ~MyRobotHardwareInterface();

  /// \brief Initialize the hardware interface
  void init();

  /// \brief Timer event
  void update(const ros::TimerEvent& e);

  /// \brief Read the state from the robot hardware.
  void read();

  /// \brief write the command to the robot hardware.
  void write(ros::Duration elapsed_time);

}; // class

} // namespace

#endif
