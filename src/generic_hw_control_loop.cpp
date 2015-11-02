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
   Desc:   Example control loop for reading, updating, and writing commands to a hardware interface
   using MONOTOIC system time
*/

#include <ros_control_boilerplate/generic_hw_control_loop.h>

namespace ros_control_boilerplate
{
GenericHWControlLoop::GenericHWControlLoop(
    ros::NodeHandle& nh,
    boost::shared_ptr<ros_control_boilerplate::GenericHWInterface> hardware_interface)
  : nh_(nh)
  , hardware_interface_(hardware_interface)
{
  // Create the controller manager
  controller_manager_.reset(
      new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

  // Get period - default to 100 hz
  nh_.param("hardware_control_loop/loop_hz", loop_hz_, 100.0);
  ROS_DEBUG_STREAM_NAMED("generic_hw_control_loop", "Using control freqency of " << loop_hz_ << " hz (for reading/writing)");

  // Get current time for use with first update
  clock_gettime(CLOCK_MONOTONIC, &last_time_);

  // Start timer
  ros::Duration update_freq = ros::Duration(1 / loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &GenericHWControlLoop::update, this);
}

void GenericHWControlLoop::update(const ros::TimerEvent& e)
{
  // Get change in time
  clock_gettime(CLOCK_MONOTONIC, &current_time_);
  elapsed_time_ = ros::Duration(current_time_.tv_sec - last_time_.tv_sec +
                                (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
  last_time_ = current_time_;
  // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "generic_hw_main","Sampled update loop with elapsed
  // time " << elapsed_time_.toSec());

  // Input
  hardware_interface_->read(elapsed_time_);

  // Control
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  hardware_interface_->write(elapsed_time_);
}

}  // namespace
