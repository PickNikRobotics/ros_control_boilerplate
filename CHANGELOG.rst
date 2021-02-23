^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_control_boilerplate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2021-02-23)
------------------
* Revert "Replaced boost with std shared_ptr"
  This reverts commit d3030f4d383895895931a9ca400497cfb72acdd9. This
  change broke the api, backporting it was an oversight.
* Add clang-format and catkin_lint checks to CI
  Backported from 638d8acb242813d298d6953d62ba36804b479242
* CMakeLists reformatting to better match catkin_lint expectations
  Unfortunately we will probably not be able to get catkin_lint to run
  warning-free, as it (correctly) detects that all rrbot_control launch
  files have some extra run dependencies we don't require for building the
  base package.
  Backported from 0db7327c2f14d9a0d3591adf24a82526c6053a37
* Replaced boost with std shared_ptr
  Backported from 9b46eaf115c16de9e453d59085b75aa9eebc3818
* Make NodeHandle a const reference
  The classes already make a copy of the NodeHandle so this change only
  make the interface more strict. It should not affect the external
  behavior of this class.
  Backported from 115b9624e60c38f3d5359ab186f00e9b0ae24be3
* Whitespace cleanup
  Backported from aea83fc71f9a05a24382b673144cc4045b7d8a1b
* test_trajectory:  Read joints list from trajectory controller params
  If the `hardware_interface/joints` list contains some joints
  configured in a trajectory controller, the `test_trajectory` program
  exits with an error:
  [ERROR]: Joints on incoming goal don't match the controller joints.
  [ INFO]: Action finished: REJECTED
  [ INFO]: TestTrajectory Finished
  [ INFO]: Shutting down.
  The correct list of joints for the trajectory controller can be found
  in the `<trajectory_controller_name>/joints` parameter.
  Since the `<trajectory_controller_name>/follow_joint_trajectory`
  parameter name is already being passed in, this patch simply shortens
  that to `<trajectory_controller_name>`, and uses it to construct both
  topic names.  Now the trajectory is generated for the correct number
  of joints.
  Backported from 33daf81d2278a193ab8b5c75e1aaf8c5667ac109
* Add Robert as maintainer to repo
  Backported from dba8338720bc3b0b2d1ed2c125953ed6d13ae3aa
* Acknowledge maintainers of this repo in README
  Backported from 5f5fca18cd987e9f6a601ce393bfd47f97758aac
* Generalize GenericHWControlLoop to all types of RobotHW (`#38 <https://github.com/PickNikRobotics/ros_control_boilerplate/issues/38>`_)
  Backported from 8353dd3f3e4df30bf86269f695769e788227b8ed
* Update README.md
  Backported from 4e08588ba61e88893a1a6ab7f800144f22cb38a3
* Fix build badges in README (`#37 <https://github.com/PickNikRobotics/ros_control_boilerplate/issues/37>`_)
  Backported from 66fd47b00b499f4e677fb47ecf8bd848b877f8e0
  There doesn't seem to be any kinetic release for debian, so it is left
  out
  Backported from 66fd47b00b499f4e677fb47ecf8bd848b877f8e0
* Update .travis.yml to use moveit_ci (`#36 <https://github.com/PickNikRobotics/ros_control_boilerplate/issues/36>`_)
  Backported from 8b50f2142a244a941b576538cf0f44879d5997d9
* Increase num AsyncSpinners where control loops are instantiated
  Backported from b6a06c856c65698340cb5251531ec0e5ee94aabc
* Merge pull request `#20 <https://github.com/PickNikRobotics/ros_control_boilerplate/issues/20>`_ from ipa-mdl/fix-loop-deadlock
  refactor GenericHWControlLoop to a sleep-based loop
* Merge pull request `#19 <https://github.com/PickNikRobotics/ros_control_boilerplate/issues/19>`_ from PaulBouchier/kinetic-devel
  change sim_control_mode to 0 (position) so demo works
* Merge pull request `#21 <https://github.com/PickNikRobotics/ros_control_boilerplate/issues/21>`_ from MohmadAyman/fix_typo
  fixed a typo in readme
* fixed a typo in readme
* refactor GenericHWControlLoop to a sleep-based loop
  using ros:Timer might lead to deadlocks
* initialize desired_update_period\_ (renamed from desired_update_freq\_)
* Revert "Depend on Eigen3"
  This reverts commit 608cc2fd64739ee56c3fbd5a0ae9d5d26b5684d0.
* change sim_control_mode to 0 (position) so demo works
* Merge pull request `#16 <https://github.com/PickNikRobotics/ros_control_boilerplate/issues/16>`_ from lucasw/xml-version
  xml version tags for all launch files.
* xml version tags for all launch files.
* Merge pull request `#15 <https://github.com/PickNikRobotics/ros_control_boilerplate/issues/15>`_ from enricotoi/kinetic-devel
  Fixed a typo in the README.md
* Fixed a typo in the README.md
  rrbot_simulaton.launch -> rrbot_simulation.launch
* Update README.md
* Contributors: AndyZe, Dave Coleman, Jafar Abdi, John Morris, Lucas Walter, Mathias Lüdtke, Paul Bouchier, Ramon Wijnands, Robert Wilbrandt, RobertWilbrandt, Tim Übelhör, enrico toivinen, mohmad ayman

0.4.1 (2017-06-20)
------------------
* Changed boost::shared_ptr to typedef for Lunar support
* Implemented simulated velocity control
* Contributors: Dave Coleman

0.4.0 (2016-06-29)
------------------
* Depend on Eigen3
* Remove dependency on meta package
* Fixed var name
* Contributors: Dave Coleman

0.3.1 (2016-01-13)
------------------
* API deprecation fix for rosparam_shortcuts
* Switched to better use of rosparam_shortcuts
* Ability to record all controller status data, not just at certain frequency
* Contributors: Dave Coleman

0.3.0 (2015-12-27)
------------------
* Removed bad reference name
* Switched to using name\_
* Record error data
* Disable soft joint limits
* header to debug output
* Added error checking of control loops time
* Fix init() bug
* Contributors: Dave Coleman

0.2.1 (2015-12-09)
------------------
* Merge branch 'indigo-devel' of github.com:davetcoleman/ros_control_boilerplate into indigo-devel
* Fix install path
* Improve user output message
* Contributors: Dave Coleman

0.2.0 (2015-12-09)
------------------
* Do not automatically call init()
* Removed warning of joint limits for continous joints
* Fix missing variable
* Improved rrbot_control example package
* Moved rrbot example code into subdirectory
* Contributors: Dave Coleman

0.1.4 (2015-12-07)
------------------
* Added missing dependency on sensor_msgs
* Contributors: Dave Coleman

0.1.3 (2015-12-05)
------------------
* Fix catkin lint errors
* Added FindGflags directly to this repo
* Minor fix
* Updated README
* Contributors: Dave Coleman

0.1.2 (2015-12-02)
------------------
* Added dependency on gflags
* Contributors: Dave Coleman

0.1.1 (2015-12-02)
------------------
* Added travis support
* Updated README
* Contributors: Dave Coleman

0.1.0 (2015-12-02)
------------------
* Initial release of ros_control_boilerplate
* Contributors: Dave Coleman
