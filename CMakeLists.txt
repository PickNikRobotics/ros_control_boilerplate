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
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs 
  LIBRARIES 
    generic_hardware_control_loop
    generic_hardware_interface
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-W -Wall -Wno-unused-parameter -fno-strict-aliasing")

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

# Control loop
add_library(generic_hardware_control_loop src/generic_hardware_control_loop.cpp)
target_link_libraries(generic_hardware_control_loop
  ${catkin_LIBRARIES}
)

# Hardware Interface
add_library(generic_hardware_interface src/generic_hardware_interface.cpp)
target_link_libraries(generic_hardware_interface
  ${catkin_LIBRARIES}
)

# Tool for analyzing controller performance
add_library(controller_state_to_csv src/controller_state_to_csv.cpp)
target_link_libraries(controller_state_to_csv
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(generic_hardware_main src/generic_hardware_main.cpp)
target_link_libraries(generic_hardware_main
  generic_hardware_interface
  generic_hardware_control_loop
  ${catkin_LIBRARIES}
)

# Test trajectory generator node
add_executable(test_trajectory src/test_trajectory.cpp)
target_link_libraries(test_trajectory
  ${catkin_LIBRARIES}
)

