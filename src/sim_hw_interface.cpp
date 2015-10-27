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
   Desc:   Example ros_control hardware interface that performs a perfect control loop for
   simulation
*/

#include <ros_control_boilerplate/sim_hw_interface.h>

namespace ros_control_boilerplate
{
SimHWInterface::SimHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("generic_hw_interface", "Loaded sim_hw_interface.");
}

void SimHWInterface::read(ros::Duration elapsed_time)
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

void SimHWInterface::write(ros::Duration elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // Send commands in different modes
  int joint_mode = 0;  // TODo

  // NOTE: the following is a "simuation" example so that this boilerplate can be run without being
  // connected to hardware
  // When converting to your robot, remove the built-in PID loop and instead let the higher leverl
  // ros_control controllers take
  // care of PID loops for you. This P-controller is only intended to mimic the delay in real
  // hardware, somewhat like a simualator
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    switch (joint_mode)
    {
      case 0:  // hardware_interface::MODE_POSITION:
        // Position - Move all the states to the commanded set points slowly
        p_error_ = joint_position_command_[i] - joint_position_[i];
        // scale the rate it takes to achieve position by a factor that is invariant to the feedback
        // loop
        joint_position_[i] += p_error_ * POSITION_STEP_FACTOR;
        break;

      case 1:  // hardware_interface::MODE_VELOCITY:
        // Position - Move all the states to the commanded set points slowly
        joint_position_[i] += joint_velocity_[i] * elapsed_time.toSec();

        // Velocity - Move all the states to the commanded set points slowly
        v_error_ = joint_velocity_command_[i] - joint_velocity_[i];
        // scale the rate it takes to achieve velocity by a factor that is invariant to the feedback
        // loop
        joint_velocity_[i] += v_error_ * VELOCITY_STEP_FACTOR;
        break;

      case 2:  // hardware_interface::MODE_EFFORT:
        ROS_ERROR_STREAM_NAMED("generic_hw_interface", "Effort not implemented yet");
        break;
    }
  }
}

}  // namespace
