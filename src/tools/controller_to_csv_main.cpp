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
   Desc:   Records ros_control ControllerState data to CSV for Matlab/etc analysis
*/

#include <ros_control_boilerplate/tools/controller_to_csv.h>

// Command line arguments
#include <gflags/gflags.h>

DEFINE_string(csv_path, "/tmp/recorded_trajectory_1.csv", "File location to save recoded data to");
DEFINE_string(topic, "/robot/position_trajectory_controller/state", "ROS topic to subscribe to");

int main(int argc, char** argv)
{
  google::SetVersionString("0.0.1");
  google::SetUsageMessage("Utility to record controller topic to a CSV");
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "controller_to_csv", ros::init_options::AnonymousName);
  ROS_INFO_STREAM_NAMED("main", "Starting ControllerToCSV...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  const std::string topic = FLAGS_topic;
  const std::string csv_path = FLAGS_csv_path;
  ros_control_boilerplate::ControllerToCSV converter(topic);
  converter.startRecording(csv_path);

  ROS_INFO_STREAM_NAMED("main","Type Ctrl-C to end and save");
  ros::spin();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
