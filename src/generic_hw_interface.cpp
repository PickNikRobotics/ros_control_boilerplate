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
   Desc:   Helper ros_control hardware interface that loads configurations
*/

#include <ros_control_boilerplate/generic_hw_interface.h>
//#include <control_toolbox/filters.h>
#include <limits>

namespace ros_control_boilerplate
{
GenericHWInterface::GenericHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) : nh_(nh)
{
  // Check if the URDF model needs to be loaded
  if (urdf_model == NULL)
    loadURDF(nh, "/robot_description");
  else
    urdf_model_ = urdf_model;

  // Initialize shared memory and interfaces here
  init();  // this implementation loads from rosparam

  ROS_INFO_NAMED("generic_hw_interface", "Loaded generic_hw_interface.");
}

void GenericHWInterface::init()
{
  ROS_INFO_STREAM_NAMED("generic_hw_interface", "Reading rosparams from namespace: " << nh_.getNamespace());

  // Get joint names
  nh_.getParam("hardware_interface/joints", joint_names_);
  if (joint_names_.size() == 0)
  {
    ROS_FATAL_STREAM_NAMED("generic_hw_interface",
                           "No joints found on parameter server for controller, did you load the proper yaml file?"
                               << " Namespace: " << nh_.getNamespace() << "/hardware_interface/joints");
    exit(-1);
  }
  num_joints_ = joint_names_.size();

  // Status
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  // Command
  joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  joint_effort_command_.resize(num_joints_, 0.0);

  // Limits
  joint_position_lower_limits_.resize(num_joints_, 0.0);
  joint_position_upper_limits_.resize(num_joints_, 0.0);
  joint_velocity_limits_.resize(num_joints_, 0.0);
  joint_effort_limits_.resize(num_joints_, 0.0);

  // Initialize interfaces for each joint
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    ROS_DEBUG_STREAM_NAMED("generic_hw_interface", "Loading joint name: " << joint_names_[joint_id]);

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

    // Add command interfaces to joints
    // TODO: decide based on transmissions?
    hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);
    position_joint_interface_.registerHandle(joint_handle_position);

    hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);
    velocity_joint_interface_.registerHandle(joint_handle_velocity);

    hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);
    effort_joint_interface_.registerHandle(joint_handle_effort);

    // Load the joint limits
    registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
  }  // end for each joint

  registerInterface(&joint_state_interface_);     // From RobotHW base class.
  registerInterface(&position_joint_interface_);  // From RobotHW base class.
  registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
  registerInterface(&effort_joint_interface_);    // From RobotHW base class.
}

void GenericHWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                                             const hardware_interface::JointHandle &joint_handle_velocity,
                                             const hardware_interface::JointHandle &joint_handle_effort,
                                             std::size_t joint_id)
{
  // Default values
  joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
  joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

  // Limits datastructures
  joint_limits_interface::JointLimits joint_limits;     // Position
  joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
  bool has_joint_limits = false;
  bool has_soft_limits = false;

  // Get limits from URDF
  if (urdf_model_ == NULL)
  {
    ROS_WARN_STREAM_NAMED("generic_hw_interface", "No URDF model loaded, unable to get joint limits");
    return;
  }

  // Get limits from URDF
  const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

  // Get main joint limits
  if (urdf_joint == NULL)
  {
    ROS_ERROR_STREAM_NAMED("generic_hw_interface", "URDF joint not found " << joint_names_[joint_id]);
    return;
  }

  // Get limits from URDF
  if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
  {
    has_joint_limits = true;
    ROS_DEBUG_STREAM_NAMED("generic_hw_interface", "Joint " << joint_names_[joint_id] << " has URDF position limits ["
                                                            << joint_limits.min_position << ", "
                                                            << joint_limits.max_position << "]");
    if (joint_limits.has_velocity_limits)
      ROS_DEBUG_STREAM_NAMED("generic_hw_interface", "Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
                                                              << joint_limits.max_velocity << "]");
  }
  else
  {
    ROS_WARN_STREAM_NAMED("generic_hw_interface", "Joint " << joint_names_[joint_id] << " does not have a URDF "
                                                                                        "position limit");
  }

  // Get limits from ROS param
  if (use_rosparam_joint_limits_)
  {
    if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits))
    {
      has_joint_limits = true;
      ROS_DEBUG_STREAM_NAMED("generic_hw_interface",
                             "Joint " << joint_names_[joint_id] << " has rosparam position limits ["
                                      << joint_limits.min_position << ", " << joint_limits.max_position << "]");
      if (joint_limits.has_velocity_limits)
        ROS_DEBUG_STREAM_NAMED("generic_hw_interface", "Joint " << joint_names_[joint_id]
                                                                << " has rosparam velocity limit ["
                                                                << joint_limits.max_velocity << "]");
    }  // the else debug message provided internally by joint_limits_interface
  }

  // Get soft limits from URDF
  if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
  {
    has_soft_limits = true;
    ROS_DEBUG_STREAM_NAMED("generic_hw_interface", "Joint " << joint_names_[joint_id] << " has soft joint limits.");
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("generic_hw_interface", "Joint " << joint_names_[joint_id] << " does not have soft joint "
                                                                                         "limits");
  }

  // Quit we we haven't found any limits in URDF or rosparam server
  if (!has_joint_limits)
  {
    return;
  }

  // Copy position limits if available
  if (joint_limits.has_position_limits)
  {
    // Slighly reduce the joint limits to prevent floating point errors
    joint_limits.min_position += std::numeric_limits<double>::epsilon();
    joint_limits.max_position -= std::numeric_limits<double>::epsilon();

    joint_position_lower_limits_[joint_id] = joint_limits.min_position;
    joint_position_upper_limits_[joint_id] = joint_limits.max_position;
  }

  // Copy velocity limits if available
  if (joint_limits.has_velocity_limits)
  {
    joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
  }

  // Copy effort limits if available
  if (joint_limits.has_effort_limits)
  {
    joint_effort_limits_[joint_id] = joint_limits.max_effort;
  }

  if (has_soft_limits)  // Use soft limits
  {
    ROS_DEBUG_STREAM_NAMED("generic_hw_interface", "Using soft saturation limits");
    const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position,
                                                                                       joint_limits, soft_limits);
    pos_jnt_soft_limits_.registerHandle(soft_handle_position);
    const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity,
                                                                                       joint_limits, soft_limits);
    vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);
    const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits,
                                                                                   soft_limits);
    eff_jnt_soft_limits_.registerHandle(soft_handle_effort);
  }
  else  // Use saturation limits
  {
    ROS_DEBUG_STREAM_NAMED("generic_hw_interface", "Using saturation limits (not soft limits)");

    const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
    pos_jnt_sat_interface_.registerHandle(sat_handle_position);

    const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
    vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);

    const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
    eff_jnt_sat_interface_.registerHandle(sat_handle_effort);
  }
}

void GenericHWInterface::enforceLimits(ros::Duration &period)
{
  // Note - you should only need to use one saturation interface,
  // depending on your control method

  // Saturation Limits

  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits
  //pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
}

void GenericHWInterface::reset()
{
  // Reset joint limits state, in case of mode switch or e-stop
  pos_jnt_sat_interface_.reset();
  pos_jnt_soft_limits_.reset();
}

void GenericHWInterface::printState()
{
  // WARNING: THIS IS NOT REALTIME SAFE
  // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
  ROS_INFO_STREAM_THROTTLE(1, std::endl
                                  << printStateHelper());
}

std::string GenericHWInterface::printStateHelper()
{
  std::stringstream ss;
  std::cout.precision(15);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ss << "j" << i << ": " << std::fixed << joint_position_[i] << "\t ";
    ss << std::fixed << joint_velocity_[i] << "\t ";
    ss << std::fixed << joint_effort_[i] << std::endl;
  }
  return ss.str();
}

std::string GenericHWInterface::printCommandHelper()
{
  std::stringstream ss;
  std::cout.precision(15);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ss << "j" << i << ": " << std::fixed << joint_position_command_[i] << "\t ";
    ss << std::fixed << joint_velocity_command_[i] << "\t ";
    ss << std::fixed << joint_effort_command_[i] << std::endl;
  }
  return ss.str();
}

void GenericHWInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
  std::string urdf_string;
  std::string robot_description = "/robot_description";
  urdf_model_ = new urdf::Model();

  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok())
  {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("generic_hw_main", "Waiting for model URDF in parameter [%s] on the ROS param server.",
                          search_param_name.c_str());
      nh.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("generic_hw_main", "Waiting for model URDF in parameter [%s] on the ROS param server.",
                          robot_description.c_str());
      nh.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }

  if (!urdf_model_->initString(urdf_string))
    ROS_ERROR_STREAM_NAMED("generic_hw_main", "Unable to load URDF model");
  else
    ROS_DEBUG_STREAM_NAMED("generic_hw_main", "Received URDF from param server");
}

}  // namespace
