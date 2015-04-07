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

#include <ros_control_boilerplate/generic_hardware_interface.h>

namespace ros_control_boilerplate
{

GenericHardwareInterface::GenericHardwareInterface(ros::NodeHandle& nh, int joint_mode)
  : nh_(nh)
  , joint_mode_(joint_mode)
{
  // Initialize shared memory and interfaces here
  init(); // this implementation loads from rosparam

  ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
}

void GenericHardwareInterface::init()
{
  ROS_WARN_STREAM_NAMED("init","Using default init function");

  // Get joint names
  nh_.getParam("hardware_interface/joints", joint_names_);
  if (joint_names_.size() == 0)
  {
    ROS_FATAL_STREAM_NAMED("init","Not joints found on parameter server for controller, did you load the proper yaml file?");
  }
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Initialize controller
  for (int i = 0; i < num_joints_; ++i)
  {
    ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint_names_[i]);

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

    switch (joint_mode_)
    {
      case 0:
        // Create position joint interface
        position_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),&joint_position_command_[i]));                                                                                 
        break;

      case 1:
        // Create velocity joint interface
        velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),&joint_velocity_command_[i]));                                                                                 
        break;

      case 2:
        // Create effort joint interface
        effort_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),&joint_effort_command_[i]));                                                                               
        break;
    }

  }
  registerInterface(&joint_state_interface_); // From RobotHW base class.
  registerInterface(&position_joint_interface_); // From RobotHW base class.
  registerInterface(&velocity_joint_interface_); // From RobotHW base class.
  registerInterface(&effort_joint_interface_); // From RobotHW base class.
}

void GenericHardwareInterface::read()
{
  // Read the joint states from your hardware here
  // e.g.
  // for (std::size_t i = 0; i < num_joints_; ++i)
  // {
  //   joint_position_[i] = robot_api_.getJointPosition(i);
  //   joint_velocity_[i] = robot_api_.getJointVelocity(i);
  //   joint_effort_[i] = robot_api_.getJointEffort(i);
  // }
}

void GenericHardwareInterface::write(ros::Duration elapsed_time)
{
  // Send commands in different modes

  // NOTE: the following is a "simuation" example so that this boilerplate can be run without being connected to hardware
  // When converting to your robot, remove the built-in PID loop and instead let the higher leverl ros_control controllers take
  // care of PID loops for you. This P-controller is only intended to mimic the delay in real hardware, somewhat like a simualator
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    switch (joint_mode_)
    {
      case 0: //hardware_interface::MODE_POSITION:
        // Position - Move all the states to the commanded set points slowly
        p_error_ = joint_position_command_[i] - joint_position_[i];
        // scale the rate it takes to achieve position by a factor that is invariant to the feedback loop
        joint_position_[i] += p_error_ * POSITION_STEP_FACTOR;
        break;

      case 1: //hardware_interface::MODE_VELOCITY:
        // Position - Move all the states to the commanded set points slowly
        joint_position_[i] += joint_velocity_[i] * elapsed_time.toSec();

        // Velocity - Move all the states to the commanded set points slowly
        v_error_ = joint_velocity_command_[i] - joint_velocity_[i];
        // scale the rate it takes to achieve velocity by a factor that is invariant to the feedback loop
        joint_velocity_[i] += v_error_ * VELOCITY_STEP_FACTOR;
        break;

      case 2: //hardware_interface::MODE_EFFORT:
        ROS_ERROR_STREAM_NAMED("write","Effort not implemented yet");
        break;
    }
  }
}


} // namespace
