^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_control_boilerplate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
