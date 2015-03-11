# ros_control Boilerplate

Provides a simple simulation interface and template for setting up a hardware interface for ros_control. This boilerplate demonstrates:

 - Creating a hardware_interface for multiple joints for use with ros_control
 - Position Trajectory Controller
 - Control of 2 joints of a simple robot
 - Loading configurations with roslaunch and yaml files
 - Generating a random trajectory and sending it over an actionlib interface
 - Partial support of joint mode switching (but this feature has not been release with ros_control yet)
 - Visualization in Rviz

Developed by [Dave Coleman](dav.ee) at the University of Colorado Boulder

<img align="right" src="https://raw.github.com/davetcoleman/ros_control_boilerplate/indigo-devel/resources/screenshot.png" />

## Video Demo

See [YouTube](https://www.youtube.com/watch?v=Tpj2tx9uZ-o) for a very modest video demo.

## Install

This package depends on [gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos) for its ``rrbot_description`` package, so be sure to ``git clone``` that along with this package and build in your catkin workspace.

## Run

This package is setup to run the "rrbot" two joint revolute-revolute robot demo. To run its ros_control hardware interface, run:

    roslaunch ros_control_boilerplate myrobot_hardware.launch

To visualize its published ``/tf`` coordinate transforms in Rviz run:

    roslaunch ros_control_boilerplate myrobot_visualize.launch

To send a random, dummy trajectory to execute, run:

    roslaunch ros_control_boilerplate myrobot_test_trajectory.launch

## Limitations

 - Does not implement joint limits, estops, transmissions, or other fancy new features of ros_contorl
 - Does not have any sort of hard realtime code, this depends largely on your platform, kernel, OS, etc
 - Only position control is fully implemented, though some code is in place for velocity and effort control
