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
   Desc:   Generate a random trajectory to test the ros_control controller
*/

// ROS
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace ros_control_boilerplate
{
static const double SEC_PER_TRAJ_POINT = 5.0;  // time between points
static const std::size_t TRAJ_POINTS = 10;     // number of points to generate

class TestTrajectory
{
public:
  /**
   * \brief Constructor
   */
  TestTrajectory()
    : nh_private_("~")
  {
    std::string action_topic;
    nh_private_.getParam("action_topic", action_topic);
    if (action_topic.empty())
    {
      ROS_FATAL_STREAM_NAMED(
          "test_trajectory",
          "Not follow joint trajectory action topic found on the parameter server");
      exit(-1);
    }
    ROS_INFO_STREAM_NAMED("test_trajectory", "Connecting to action " << action_topic);

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client(
        action_topic, true);

    ROS_INFO_NAMED("test_trajetory", "Waiting for action server to start.");
    // wait for the action server to start
    action_client.waitForServer();  // will wait for infinite time

    ROS_INFO_NAMED("test_trajetory", "Action server started, sending goal.");

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = createTrajectory();
    std::cout << "Trajectry:\n" << goal.trajectory << std::endl;
    action_client.sendGoal(goal);

    // Wait for the action to return
    double wait_extra_padding = 2;  // time to wait longer than trajectory itself
    bool finished_before_timeout = action_client.waitForResult(
        ros::Duration(goal.trajectory.points.back().time_from_start.toSec() + wait_extra_padding));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = action_client.getState();
      ROS_INFO_NAMED("test_trajetory", "Action finished: %s", state.toString().c_str());
    }
    else
      ROS_INFO_NAMED("test_trajetory", "Action did not finish before the time out.");

    ROS_INFO_STREAM_NAMED("test_trajectory", "TestTrajectory Finished");
  }

  /**
   * \brief Create random trajectory
   */
  trajectory_msgs::JointTrajectory createTrajectory()
  {
    std::vector<std::string> joint_names;
    double min_joint_value = -3.14;
    double max_joint_value = 3.14;

    // Get joint names
    nh_private_.getParam("hardware_interface/joints", joint_names);
    if (joint_names.size() == 0)
    {
      ROS_FATAL_STREAM_NAMED(
          "init",
          "Not joints found on parameter server for controller, did you load the proper yaml file?"
              << " Namespace: " << nh_private_.getNamespace() << "/hardware_interface/joints");
      exit(-1);
    }

    nh_private_.getParam("min_joint_value", min_joint_value);
    nh_private_.getParam("max_joint_value", max_joint_value);
    ROS_DEBUG_STREAM_NAMED("test_trajectory", "Creating trajectory with joint values from "
                                                  << min_joint_value << " to " << max_joint_value);

    // Create header
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.joint_names = joint_names;

    // Create trajectory with x points
    trajectory.points.resize(TRAJ_POINTS);
    for (std::size_t i = 0; i < TRAJ_POINTS; ++i)
    {
      trajectory.points[i].positions.resize(joint_names.size());
      // for each joint
      for (std::size_t j = 0; j < joint_names.size(); ++j)
      {
        trajectory.points[i].positions[j] = dRand(min_joint_value, max_joint_value);
        trajectory.points[i].time_from_start = ros::Duration(i * SEC_PER_TRAJ_POINT);
      }
    }

    return trajectory;
  }

  /** \brief Get random number */
  double dRand(double dMin, double dMax)
  {
    double d = (double)rand() / RAND_MAX;
    return dMin + d * (dMax - dMin);
  }

private:
  // A shared node handle
  ros::NodeHandle nh_private_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TestTrajectory> TestTrajectoryPtr;
typedef boost::shared_ptr<const TestTrajectory> TestTrajectoryConstPtr;

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectory");
  ROS_INFO_STREAM_NAMED("test_trajectory", "Starting TestTrajectory...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros_control_boilerplate::TestTrajectory server;

  ROS_INFO_STREAM_NAMED("test_trajectory", "Shutting down.");
  ros::shutdown();

  return 0;
}
