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
  urdf
  joint_limits_interface
  transmission_interface
  control_toolbox
  std_msgs
  gflags
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
    urdf
    joint_limits_interface
    transmission_interface
    control_toolbox
    std_msgs
  LIBRARIES 
    generic_hw_control_loop
    sim_hw_interface
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

# Control loop
add_library(generic_hw_control_loop src/generic_hw_control_loop.cpp)
target_link_libraries(generic_hw_control_loop
  ${catkin_LIBRARIES}
)

# Simulation Hardware Interface
add_library(sim_hw_interface 
  src/generic_hw_interface.cpp 
  src/sim_hw_interface.cpp
)
target_link_libraries(sim_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(sim_hw_main src/sim_hw_main.cpp)
target_link_libraries(sim_hw_main
  sim_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

# Test trajectory generator node
add_executable(test_trajectory src/tools/test_trajectory.cpp)
target_link_libraries(test_trajectory
  ${catkin_LIBRARIES}
)

## TOOLS ------------------------------------------------------

# Tool for analyzing controller performance
add_library(controller_to_csv src/tools/controller_to_csv.cpp)
target_link_libraries(controller_to_csv
  ${catkin_LIBRARIES}
)
add_executable(controller_to_csv_main src/tools/controller_to_csv_main.cpp)
target_link_libraries(controller_to_csv_main
  controller_to_csv
  ${catkin_LIBRARIES}
)

# Tool for analyzing controller performance
add_library(csv_to_controller src/tools/csv_to_controller.cpp)
target_link_libraries(csv_to_controller
  ${catkin_LIBRARIES}
)
add_executable(csv_to_controller_main src/tools/csv_to_controller_main.cpp)
target_link_libraries(csv_to_controller_main
  csv_to_controller
  ${catkin_LIBRARIES}
)

add_executable(keyboard_teleop src/tools/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop
  ${catkin_LIBRARIES}
)

