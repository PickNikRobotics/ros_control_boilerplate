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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Inherit from this file to enable joystick mode switching of your robot
*/

#ifndef ROS_CONTROL_BOILERPLATE__JOYSTICK_MANUAL_CONTROL
#define ROS_CONTROL_BOILERPLATE__JOYSTICK_MANUAL_CONTROL

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

// ros_control
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>

namespace ros_control_boilerplate
{
class JoystickManualControl
{
public:
  /**
   * \brief Constructor
   * \param parent_name - name of parent class, only used for namespacing logging debug output
   * \param service_namespace - prefix to controller manager service, or blank. do not add trailing
   * "/"
   */
  JoystickManualControl(const std::string& parent_name, const std::string& service_namespace)
    : parent_name_(parent_name)
    , using_trajectory_controller_(true)
  {
    switch_service_ = service_namespace + "/controller_manager/switch_controller";
    load_service_ = service_namespace + "/controller_manager/load_controller";

    // Switch modes of controllers
    switch_controlers_client_ =
        nh_.serviceClient<controller_manager_msgs::SwitchController>(switch_service_);
    load_controlers_client_ =
        nh_.serviceClient<controller_manager_msgs::LoadController>(load_service_);

    // Subscribe to joystick control
    std::size_t queue_size = 1;
    remote_joy_ = nh_.subscribe("/joy", queue_size, &JoystickManualControl::joyCallback, this);

    ROS_INFO_STREAM_NAMED(parent_name_, "JoystickManualControl Ready.");
  }

  /**
   * \brief Response to joystick control
   *        Button mapping is customized by each class that inherits from this
   */
  virtual void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) = 0;

  /**
   * \brief Load a secondary manual controller
   */
  bool loadManualControllers()
  {
    // Ensure services are up
    ROS_INFO_STREAM_NAMED(parent_name_, "Waiting for serivces...");
    if (!ros::service::waitForService(switch_service_, ros::Duration(10)))
      ROS_ERROR_STREAM_NAMED(parent_name_, "Unable to find service " << switch_service_);
    if (!ros::service::waitForService(load_service_, ros::Duration(10)))
      ROS_ERROR_STREAM_NAMED(parent_name_, "Unable to find service " << load_service_);

    for (std::size_t i = 0; i < manual_controllers_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(parent_name_, "Loading controller " << manual_controllers_[i]);
      controller_manager_msgs::LoadController service;
      service.request.name = manual_controllers_[i];
      std::size_t counter = 0;
      while (!load_controlers_client_.call(service) && ros::ok())
      {
        if (counter > 100)
          ROS_WARN_STREAM_THROTTLE_NAMED(1.0, parent_name_, "Failed to load controller '"
                                                                << manual_controllers_[i]
                                                                << "', trying again");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        counter++;
      }
    }

    return true;
  }

  void switchToManual()
  {
    // Stop all controllers, soft E-Stop
    controller_manager_msgs::SwitchController service;
    service.request.strictness = service.request.STRICT;

    ROS_WARN_STREAM_NAMED(parent_name_, "Switching to MANUAL control");
    for (std::size_t i = 0; i < manual_controllers_.size(); ++i)
    {
      service.request.start_controllers.push_back(manual_controllers_[i]);
    }
    for (std::size_t i = 0; i < trajectory_controllers_.size(); ++i)
    {
      service.request.stop_controllers.push_back(trajectory_controllers_[i]);
    }

    // Attempt stop
    if (!switch_controlers_client_.call(service))
    {
      ROS_ERROR_STREAM_NAMED(parent_name_, "Failed to switch controllers");
      return;
    }
  }

  void switchToTrajectory()
  {
    // Stop all controllers, soft E-Stop
    controller_manager_msgs::SwitchController service;
    service.request.strictness = service.request.STRICT;

    ROS_INFO_STREAM_NAMED(parent_name_, "Switching to TRAJECTORY control");
    for (std::size_t i = 0; i < manual_controllers_.size(); ++i)
    {
      service.request.stop_controllers.push_back(manual_controllers_[i]);
    }
    for (std::size_t i = 0; i < trajectory_controllers_.size(); ++i)
    {
      service.request.start_controllers.push_back(trajectory_controllers_[i]);
    }

    // Attempt stop
    if (!switch_controlers_client_.call(service))
    {
      ROS_ERROR_STREAM_NAMED(parent_name_, "Failed to switch controllers");
      return;
    }
  }

protected:
  // A shared node handle
  ros::NodeHandle nh_;

  // Name of parent class, used for logging messages
  const std::string parent_name_;
  std::string switch_service_;
  std::string load_service_;

  // Subscribe to joystick commands
  ros::Subscriber remote_joy_;

  // Ability to switch controllers
  ros::ServiceClient switch_controlers_client_;
  ros::ServiceClient load_controlers_client_;

  // Switching controller mode
  bool using_trajectory_controller_;

  // Controller lists
  std::vector<std::string> manual_controllers_;
  std::vector<std::string> trajectory_controllers_;

};  // end class

}  // end namespace

#endif
