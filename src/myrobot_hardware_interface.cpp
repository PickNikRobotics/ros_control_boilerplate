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

#include <myrobot_ros_control/myrobot_hardware_interface.h>

namespace ros_control_boilerplate
{

MyRobotHardwareInterface::MyRobotHardwareInterface(ros::NodeHandle& nh)
  : nh_(nh)
  , joint_mode_(1) // POSITION
{
  // Initialize hardware communication connection
  // DO THIS HERE

  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

  // Get period and create timer
  nh_.param("loop_hz", loop_hz_, 0.1);
  ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &MyRobotHardwareInterface::update, this);

  ROS_INFO_NAMED("hardware_interface", "Loaded myrobot_hardware_interface.");
}

MyRobotHardwareInterface::~MyRobotHardwareInterface()
{
}

void MyRobotHardwareInterface::init()
{
  // Get joints to control
  XmlRpc::XmlRpcValue joint_list;
  nh_.getParam("joints", joint_list);
  if (joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    for (int32_t i = 0; i < joint_list.size(); ++i)
    {
      ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      joint_names_.push_back(static_cast<std::string>(joint_list[i]));
    }
  else
    ROS_ERROR_STREAM_NAMED("temp","joint list type is not type array???");

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
    // Create joint name
    //joint_names_.push_back("TODO");

    ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint_names_[i]);

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

    if (i < 6 )
    {
      // Create velocity joint interface
      velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(
                                                                               joint_state_interface_.getHandle(joint_names_[i]),&joint_velocity_command_[i]));
    }
    else // this is the gripper
    {
      // Create position joint interface
      position_joint_interface_.registerHandle(hardware_interface::JointHandle(
                                                                               joint_state_interface_.getHandle(joint_names_[i]),&joint_position_command_[i]));
    }

    // Create effort joint interface
    //effort_joint_interface_.registerHandle(hardware_interface::JointHandle(
    //    joint_state_interface_.getHandle(joint_names_[i]),&joint_effort_command_[i]));

  }
  registerInterface(&joint_state_interface_); // From RobotHW base class.
  registerInterface(&position_joint_interface_); // From RobotHW base class.
  registerInterface(&velocity_joint_interface_); // From RobotHW base class.
  registerInterface(&effort_joint_interface_); // From RobotHW base class.
}

void MyRobotHardwareInterface::update(const ros::TimerEvent& e)
{
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);

  // Input
  read();

  // Control
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  write(elapsed_time_);
}

void MyRobotHardwareInterface::read()
{
}

void MyRobotHardwareInterface::write(ros::Duration elapsed_time)
{
  // Send commands in different modes

  // Move all the states to the commanded set points slowly
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    switch (joint_mode_)
    {
      case 1: //hardware_interface::MODE_POSITION:
        // Position
        std::cout << "position mode " << std::endl;
        p_error_ = joint_position_command_[i] - joint_position_[i];
        // scale the rate it takes to achieve position by a factor that is invariant to the feedback loop
        joint_position_[i] += p_error_ * POSITION_STEP_FACTOR / loop_hz_;
        break;

      case 2: //hardware_interface::MODE_VELOCITY:
        std::cout << "velocity mode " << std::endl;
        // Position
        joint_position_[i] += joint_velocity_[i] * elapsed_time.toSec();

        // Velocity
        v_error_ = joint_velocity_command_[i] - joint_velocity_[i];
        // scale the rate it takes to achieve velocity by a factor that is invariant to the feedback loop
        joint_velocity_[i] += v_error_ * VELOCITY_STEP_FACTOR / loop_hz_;

        break;
      case 3: //hardware_interface::MODE_EFFORT:
        ROS_ERROR_STREAM_NAMED("write","Effort not implemented yet");
        break;
    }
  }
}


} // namespace
