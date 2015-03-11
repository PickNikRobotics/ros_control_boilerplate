cmake_minimum_required(VERSION 2.8.3)
project(ros_control_boilerplate)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS hardware_interface roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)

# ROS Node
add_executable(generic_hardware_interface src/generic_hardware_interface.cpp)
target_link_libraries(generic_hardware_interface
    ${catkin_LIBRARIES}
)

# Test trajectory generator node
add_executable(test_trajectory src/test_trajectory.cpp)
target_link_libraries(test_trajectory
    ${catkin_LIBRARIES}
)

