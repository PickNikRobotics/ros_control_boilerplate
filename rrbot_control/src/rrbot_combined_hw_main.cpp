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
   Desc:   Example ros_control main() entry point for controlling robots using
   a combined_robot_hw in ROS
*/

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <combined_robot_hw/combined_robot_hw.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrbot_combined_hw_interface");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the combined_robot_hw
  std::shared_ptr<combined_robot_hw::CombinedRobotHW> combined_hw_interface(new combined_robot_hw::CombinedRobotHW());
  combined_hw_interface->init(nh, priv_nh);

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, combined_hw_interface);
  control_loop.run();  // Blocks until shutdown signal recieved

  return 0;
}
