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
   Desc:   Records a ros_control ControllerState data to CSV for Matlab/etc analysis
*/

// ROS
#include <ros/ros.h>

// ros_control
#include <control_msgs/FollowJointTrajectoryAction.h>

// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Basic file operations
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iterator>
#include <vector>
#include <string>

namespace kinova_control
{

static const double RECORD_RATE_HZ = 100.0; // times per second to record

class CSVToController
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  CSVToController(bool verbose)
    : verbose_(verbose)
    , joint_trajectory_action_("/kinova/velocity_trajectory_controller/follow_joint_trajectory")
  {

    ROS_INFO_STREAM_NAMED("csv_to_controller","Waiting for action server");
    joint_trajectory_action_.waitForServer();

    ROS_INFO_STREAM_NAMED("csv_to_controller","CSVToController Ready.");
  }

  // Start the data collection
  void loadAndRunCSV(const std::string& file_name)
  {
    file_name_ = file_name;

    // Open file
    std::ifstream input_file;
    input_file.open (file_name_.c_str());

    // Settings
    double num_joints = 6;
    std::string line;
    std::string cell;
    double value;

    control_msgs::FollowJointTrajectoryGoal goal;

    // Populate joint names
    goal.trajectory.joint_names.push_back("jaco2_joint_1");
    goal.trajectory.joint_names.push_back("jaco2_joint_2");
    goal.trajectory.joint_names.push_back("jaco2_joint_3");
    goal.trajectory.joint_names.push_back("jaco2_joint_4");
    goal.trajectory.joint_names.push_back("jaco2_joint_5");
    goal.trajectory.joint_names.push_back("jaco2_joint_6");

    // Skip header
    std::getline(input_file, line);

    // For each line/row
    while(std::getline(input_file, line))
    {
      std::stringstream lineStream(line);

      trajectory_msgs::JointTrajectoryPoint point;

      // TIME FROM START
      if(!std::getline(lineStream, cell,','))
        ROS_ERROR_STREAM_NAMED("temp","no joint value");

      point.time_from_start = ros::Duration(atof(cell.c_str()));

      // For each item/column
      for (std::size_t i = 0; i < num_joints; ++i)
      {
        // DESIRED POSITION
        if(!std::getline(lineStream, cell,','))
          ROS_ERROR_STREAM_NAMED("temp","no joint value");

        point.positions.push_back(atof(cell.c_str()));

        // DESIRED VELOCITY
        if(!std::getline(lineStream, cell,','))
          ROS_ERROR_STREAM_NAMED("temp","no joint value");

        point.velocities.push_back(atof(cell.c_str()));

        // ACTUAL POSITION
        if(!std::getline(lineStream, cell,','))
          ROS_ERROR_STREAM_NAMED("temp","no joint value");
        // ACTUAL VELOCITY
        if(!std::getline(lineStream, cell,','))
          ROS_ERROR_STREAM_NAMED("temp","no joint value");
        // COMMANDED VELOCITY
        if(!std::getline(lineStream, cell,','))
          ROS_ERROR_STREAM_NAMED("temp","no joint value");
      }

      goal.trajectory.points.push_back(point);
    } // while

    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);

    std::cout << "TRAJECTORY: " << goal.trajectory  << std::endl;
    joint_trajectory_action_.sendGoal(goal);    
  }

private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Action
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_trajectory_action_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // Where to save the CSV
  std::string file_name_;

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "csv_to_controller");
  ROS_INFO_STREAM_NAMED("main", "Starting CSVToController...");

  // Get file name
  if (argc < 2)
  {
    ROS_ERROR_STREAM_NAMED("csv_to_controller","No file name passed in");
    return 0;
  }
  std::string path = argv[1];
  ROS_INFO_STREAM_NAMED("csv_to_controller","Reading from file " << path);

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
        continue;
      }
    }
  }

  kinova_control::CSVToController converter(verbose);
  converter.loadAndRunCSV(path);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
