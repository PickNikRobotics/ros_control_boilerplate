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

#ifndef GENERIC_ROS_CONTROL__CSV_TO_CONTROLLER_H
#define GENERIC_ROS_CONTROL__CSV_TO_CONTROLLER_H

// ROS
#include <ros/ros.h>

// ros_control
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>

// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace ros_control_boilerplate
{
static const double RECORD_RATE_HZ = 100.0;  // times per second to record

class CSVToController
{
public:
  /**
   * \brief Constructor
   */
  CSVToController(const std::string& joint_trajectory_action,
                  const std::string& controller_state_topic);

  /** \brief Callback from ROS message */
  void stateCB(const control_msgs::JointTrajectoryControllerState::ConstPtr& state);

  void printPoint(trajectory_msgs::JointTrajectoryPoint &point);

  // Start the data collection
  void loadAndRunCSV(const std::string& file_name);

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Listener to state of controller
  ros::Subscriber state_sub_;

  // Action
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_trajectory_action_;

  // Where to save the CSV
  std::string file_name_;
  std::string controller_state_topic_;

  // Cache of last recieved state
  control_msgs::JointTrajectoryControllerState current_state_;

};  // end class

}  // end namespace

#endif
