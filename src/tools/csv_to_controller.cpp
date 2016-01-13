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

#include <ros_control_boilerplate/tools/csv_to_controller.h>

// Basic file operations
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iterator>
#include <vector>
#include <string>

namespace ros_control_boilerplate
{

CSVToController::CSVToController(const std::string& joint_trajectory_action,
                                 const std::string& controller_state_topic)
  : joint_trajectory_action_(joint_trajectory_action)
  , controller_state_topic_(controller_state_topic)
{
  ROS_INFO_STREAM_NAMED("csv_to_controller", "Waiting for action server");
  joint_trajectory_action_.waitForServer();

  // State subscriber
  state_sub_ = nh_.subscribe<control_msgs::JointTrajectoryControllerState>(
      controller_state_topic_, 1, &CSVToController::stateCB, this);

  // Wait for states to populate
  ros::spinOnce();
  ros::Duration(0.5).sleep();

  ROS_INFO_STREAM_NAMED("csv_to_controller", "CSVToController Ready.");
}

void CSVToController::stateCB(
    const control_msgs::JointTrajectoryControllerState::ConstPtr& state)
{
  current_state_ = *state;
}

void CSVToController::printPoint(trajectory_msgs::JointTrajectoryPoint &point)
{
    // Show new line
    std::copy(point.positions.begin(), point.positions.end(), std::ostream_iterator<double>(std::cout, " "));
    std::cout << std::endl;
}

// Start the data collection
void CSVToController::loadAndRunCSV(const std::string& file_name)
{
  file_name_ = file_name;

  // Open file
  std::ifstream input_file;
  input_file.open(file_name_.c_str());

  // Settings
  std::string line;
  std::string cell;

  control_msgs::FollowJointTrajectoryGoal pre_goal; // moving to start state
  control_msgs::FollowJointTrajectoryGoal goal; // csv file

  // Populate joint names
  goal.trajectory.joint_names.push_back("joint_a1");
  goal.trajectory.joint_names.push_back("joint_a2");
  goal.trajectory.joint_names.push_back("joint_a3");
  goal.trajectory.joint_names.push_back("joint_a4");
  goal.trajectory.joint_names.push_back("joint_a5");
  goal.trajectory.joint_names.push_back("joint_a6");
  goal.trajectory.joint_names.push_back("joint_a7");
  pre_goal.trajectory.joint_names = goal.trajectory.joint_names;
  double num_joints = goal.trajectory.joint_names.size();

  // Skip header
  std::getline(input_file, line);

  // For each line/row
  while (std::getline(input_file, line))
  {
    std::stringstream lineStream(line);

    trajectory_msgs::JointTrajectoryPoint point;

    // TIME FROM START
    if (!std::getline(lineStream, cell, ','))
      ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");

    point.time_from_start = ros::Duration(atof(cell.c_str()));

    // For each item/column
    for (std::size_t i = 0; i < num_joints; ++i)
    {
      // DESIRED POSITION
      if (!std::getline(lineStream, cell, ','))
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
      // UNUSED

      // DESIRED VELOCITY
      if (!std::getline(lineStream, cell, ','))
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
      // UNUSED

      // ACTUAL POSITION
      if (!std::getline(lineStream, cell, ','))
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
      point.positions.push_back(atof(cell.c_str()));

      // ACTUAL VELOCITY
      if (!std::getline(lineStream, cell, ','))
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
      point.velocities.push_back(atof(cell.c_str()));

      // COMMANDED VEL
      if (!std::getline(lineStream, cell, ','))
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
      // UNUSED
    }

    goal.trajectory.points.push_back(point);
  }  // while

  // Check that we have a current state
  if (current_state_.actual.positions.empty())
  {
    ROS_ERROR_STREAM_NAMED("csv_to_controller","Unable to find current state msg");
    return;
  }
  // Add current state to start of trajectory
  trajectory_msgs::JointTrajectoryPoint last_point;
  last_point.positions = current_state_.actual.positions;
  last_point.velocities = current_state_.actual.velocities;

  std::cout << "Current State:" << std::endl;
  printPoint(last_point);
  printPoint(goal.trajectory.points.front());
  std::cout << "^^ Goal point " << std::endl;
  pre_goal.trajectory.points.push_back(last_point);

  // Interpolate from first point
  bool done = false;
  double max_velocity = 0.1; // m/s  or radians/s
  double frequency = 200; // hz
  double q_delta = max_velocity / frequency; // m
  double t_delta = 1/frequency;
  ros::Duration time_from_start(1);
  while(!done)
  {
    done = true;
    trajectory_msgs::JointTrajectoryPoint new_point = last_point;

    // Time change
    time_from_start += ros::Duration(t_delta);
    new_point.time_from_start = time_from_start;

    // Position change
    for (std::size_t i = 0; i < num_joints; ++i) // each joint
    {
      // Do we need to move this joint foward?
      if (new_point.positions[i] < goal.trajectory.points.front().positions[i])
      {
        // Do not allow to go past goal point
        new_point.positions[i] = std::min(new_point.positions[i] + q_delta,
                                          goal.trajectory.points.front().positions[i]);
        new_point.velocities[i] = max_velocity;
        done = false;
      } else {
        // Maintain velocity
        new_point.velocities[i] = 0;
      }
    }
    pre_goal.trajectory.points.push_back(new_point);
    last_point = new_point;

    printPoint(new_point);
  }

  //std::cout << "TRAJECTORY: " << trajectory << std::endl;
  //std::cout << "TRAJECTORY: " << goal.trajectory << std::endl;

  ROS_INFO_STREAM_NAMED("temp","Sleeping for " << time_from_start.toSec());
  pre_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
  joint_trajectory_action_.sendGoal(pre_goal);
  ros::Duration(time_from_start).sleep();

  for (std::size_t i = 0; i < goal.trajectory.points.size(); ++i)
  {
    printPoint(goal.trajectory.points[i]);
  }

  ROS_INFO_STREAM_NAMED("csv_to_controller","Preparing to follow CSV path");
  ros::Duration(0.5).sleep();
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
  joint_trajectory_action_.sendGoal(goal);
}

} // end namespace
