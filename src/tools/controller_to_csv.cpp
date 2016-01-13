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

#include <ros_control_boilerplate/tools/controller_to_csv.h>

// Basic file operations
#include <iostream>
#include <fstream>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace ros_control_boilerplate
{

ControllerToCSV::ControllerToCSV(const std::string& topic)
  : nh_("~")
  , first_update_(true)
  , recording_started_(true)
{
  // Load rosparams
  ros::NodeHandle rpsnh(nh_, name_);
  int error = 0;
  error += !rosparam_shortcuts::get(name_, rpsnh, "record_hz", record_hz_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  ROS_INFO_STREAM_NAMED(name_, "Subscribing to " << topic);
  // State subscriber
  state_sub_ = nh_.subscribe<control_msgs::JointTrajectoryControllerState>(
      topic, 1, &ControllerToCSV::stateCB, this);

  // Wait for states to populate
  waitForSubscriber(state_sub_);

  // Alert user to mode
  if (recordAll())
  {
    ROS_INFO_STREAM_NAMED(name_, "Recording all incoming controller state messages");
  }
  else
  {
    ROS_INFO_STREAM_NAMED(name_, "Only recording every " << record_hz_ << " hz");
  }

  ROS_INFO_STREAM_NAMED(name_, "ControllerToCSV Ready.");
}

ControllerToCSV::~ControllerToCSV()
{
  stopRecording();
}

bool ControllerToCSV::recordAll()
{
  return record_hz_ == 0;
}

void ControllerToCSV::stateCB(const control_msgs::JointTrajectoryControllerState::ConstPtr& state)
{
  // Two modes - save all immediately, or only save at certain frequency
  if (recordAll())
  {
    // Record state
    states_.push_back(current_state_);

    // Record current time
    timestamps_.push_back(ros::Time::now());
  }
  else // only record at freq
  {
    current_state_ = *state;
  }
}

// Start the data collection
void ControllerToCSV::startRecording(const std::string& file_name)
{
  ROS_INFO_STREAM_NAMED(name_, "Saving to " << file_name);
  file_name_ = file_name;

  // Reset data collections
  states_.clear();
  timestamps_.clear();

  recording_started_ = true;

  // Start sampling loop
  if (!recordAll()) // only record at the specified frequency
  {
    ros::Duration update_freq = ros::Duration(1.0 / record_hz_);
    non_realtime_loop_ = nh_.createTimer(update_freq, &ControllerToCSV::update, this);
  }
}

void ControllerToCSV::update(const ros::TimerEvent& event)
{
  if (first_update_)
  {
    // Check if we've recieved any states yet
    if (current_state_.joint_names.empty())
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, name_, "No states recieved yet");
      return;
    }
    first_update_ = false;
  }
  else // if (event.last_real > 0)
  {
    const double freq = 1.0 / (event.current_real - event.last_real).toSec();
    ROS_INFO_STREAM_THROTTLE_NAMED(2, name_, "Updating at " << freq << " hz, total: " << (ros::Time::now() - timestamps_.front()).toSec() << " seconds");
  }
  // Record state
  states_.push_back(current_state_);

  // Record current time
  timestamps_.push_back(ros::Time::now());
}

void ControllerToCSV::stopRecording()
{
  non_realtime_loop_.stop();
  writeToFile();
}

bool ControllerToCSV::writeToFile()
{
  std::cout << "Writing data to file " << std::endl;

  if (!states_.size())
  {
    std::cout << "No controller states populated" << std::endl;
    return false;
  }

  std::ofstream output_file;
  output_file.open(file_name_.c_str());

  // Output header -------------------------------------------------------
  output_file << "timestamp,";
  for (std::size_t j = 0; j < states_[0].joint_names.size(); ++j)
  {
    output_file << states_[0].joint_names[j] << "_desired_pos,"
                << states_[0].joint_names[j] << "_desired_vel,"
                << states_[0].joint_names[j] << "_actual_pos,"
                << states_[0].joint_names[j] << "_actual_vel,"
                << states_[0].joint_names[j] << "_error_pos,"
                << states_[0].joint_names[j] << "_error_vel,";
  }
  output_file << std::endl;

  // Output data ------------------------------------------------------

  // Subtract starting time
  // double start_time = states_[0].header.stamp.toSec();
  double start_time = timestamps_[0].toSec();

  for (std::size_t i = 0; i < states_.size(); ++i)
  {
    // Timestamp
    output_file << timestamps_[i].toSec() - start_time << ",";

    // Output entire state of robot to single line
    for (std::size_t j = 0; j < states_[i].joint_names.size(); ++j)
    {
      // Output State
      output_file << states_[i].desired.positions[j] << ","
                  << states_[i].desired.velocities[j] << ","
                  << states_[i].actual.positions[j] << ","
                  << states_[i].actual.velocities[j] << ","
                  << states_[i].error.positions[j] << ","
                  << states_[i].error.velocities[j] << ",";
    }

    output_file << std::endl;
  }
  output_file.close();
  std::cout << "Wrote to file: " << file_name_ << std::endl;
  return true;
}

bool ControllerToCSV::waitForSubscriber(const ros::Subscriber& sub, const double& wait_time)
{
  // Benchmark runtime
  ros::Time start_time;
  start_time = ros::Time::now();

  // Will wait at most 1000 ms (1 sec)
  ros::Time max_time(ros::Time::now() + ros::Duration(wait_time));

  // This is wrong. It returns only the number of subscribers that have already established their
  // direct connections to this subscriber
  int num_existing_subscribers = sub.getNumPublishers();

  // How often to check for subscribers
  ros::Rate poll_rate(200);

  // Wait for subsriber
  while (num_existing_subscribers == 0)
  {
    // Check if timed out
    if (ros::Time::now() > max_time)
    {
      ROS_WARN_STREAM_NAMED(name_, "Topic '" << sub.getTopic() << "' unable to connect to any publishers within "
                                                      << wait_time << " seconds.");
      return false;
    }
    ros::spinOnce();

    // Sleep
    poll_rate.sleep();

    // Check again
    num_existing_subscribers = sub.getNumPublishers();
  }

  // Benchmark runtime
  if (true)
  {
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_DEBUG_STREAM_NAMED(name_, "Topic '" << sub.getTopic() << "' took " << duration
                                                     << " seconds to connect to a subscriber. "
                                                        "Connected to " << num_existing_subscribers
                                                     << " total subsribers");
  }
  return true;
}

}  // end namespace
