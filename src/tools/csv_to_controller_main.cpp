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
   Desc:   Records ros_control ControllerState datas to CSV for Matlab/etc analysis
*/

#include <ros_control_boilerplate/tools/csv_to_controller.h>

// Command line arguments
#include <gflags/gflags.h>

DEFINE_string(csv_path, "", "File location to load recoded data from");
DEFINE_string(joint_trajectory_action, "", "Which action server to send commands to");
DEFINE_string(controller_state_topic, "", "Where to subscribe the controller state");

int main(int argc, char** argv)
{
  google::SetVersionString("0.0.1");
  google::SetUsageMessage("Utility to load commands from a CSV");
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "csv_to_controller");
  ROS_INFO_STREAM_NAMED("main", "Starting CSVToController...");

  // Get file name
  if (FLAGS_csv_path.empty())
  {
    ROS_ERROR_STREAM_NAMED("csv_to_controller","No file name passed in");
    return 0;
  }
  ROS_INFO_STREAM_NAMED("csv_to_controller","Reading from file " << FLAGS_csv_path);

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros_control_boilerplate::CSVToController converter(FLAGS_joint_trajectory_action, FLAGS_controller_state_topic);
  converter.loadAndRunCSV(FLAGS_csv_path);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
