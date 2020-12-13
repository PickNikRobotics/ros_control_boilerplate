cmake_minimum_required(VERSION 3.0.2)
project(ros_control_boilerplate)

# C++ 11
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  control_msgs
  control_toolbox
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  trajectory_msgs
  transmission_interface
  urdf
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    control_msgs
    control_toolbox
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    trajectory_msgs
    transmission_interface
    urdf
  LIBRARIES
    generic_hw_control_loop
    generic_hw_interface
    sim_hw_interface
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Gflags_INCLUDE_DIRS}
)

# Control loop
add_library(generic_hw_control_loop
  src/generic_hw_control_loop.cpp
)
target_link_libraries(generic_hw_control_loop
  ${catkin_LIBRARIES}
)

# Generic Hardware Interface
add_library(generic_hw_interface
  src/generic_hw_interface.cpp
)
target_link_libraries(generic_hw_interface
  ${catkin_LIBRARIES}
)

# Simulation Hardware Interface
add_library(sim_hw_interface
  src/sim_hw_interface.cpp
)
target_link_libraries(sim_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(${PROJECT_NAME}_sim_hw_main src/sim_hw_main.cpp)
set_target_properties(${PROJECT_NAME}_sim_hw_main PROPERTIES OUTPUT_NAME sim_hw_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_sim_hw_main
  sim_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

# Test trajectory generator node
add_executable(${PROJECT_NAME}_test_trajectory src/tools/test_trajectory.cpp)
set_target_properties(${PROJECT_NAME}_test_trajectory PROPERTIES OUTPUT_NAME test_trajectory PREFIX "")
target_link_libraries(${PROJECT_NAME}_test_trajectory
  ${catkin_LIBRARIES}
)

## TOOLS ------------------------------------------------------

# Tool for analyzing controller performance
add_library(controller_to_csv src/tools/controller_to_csv.cpp)
target_link_libraries(controller_to_csv
  ${catkin_LIBRARIES}
)
add_executable(${PROJECT_NAME}_controller_to_csv_main src/tools/controller_to_csv_main.cpp)
set_target_properties(${PROJECT_NAME}_controller_to_csv_main PROPERTIES OUTPUT_NAME controller_to_csv_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_controller_to_csv_main
  controller_to_csv
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for analyzing controller performance
add_library(csv_to_controller src/tools/csv_to_controller.cpp)
target_link_libraries(csv_to_controller
  ${catkin_LIBRARIES}
)
add_executable(${PROJECT_NAME}_csv_to_controller_main src/tools/csv_to_controller_main.cpp)
set_target_properties(${PROJECT_NAME}_csv_to_controller_main PROPERTIES OUTPUT_NAME csv_to_controller_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_csv_to_controller_main
  csv_to_controller
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for controlling a robot from keyboard
add_executable(${PROJECT_NAME}_keyboard_teleop src/tools/keyboard_teleop.cpp)
set_target_properties(${PROJECT_NAME}_keyboard_teleop PROPERTIES OUTPUT_NAME keyboard_teleop PREFIX "")
target_link_libraries(${PROJECT_NAME}_keyboard_teleop
  ${catkin_LIBRARIES}
)

add_subdirectory(rrbot_control)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    controller_to_csv
    csv_to_controller
    generic_hw_control_loop
    generic_hw_interface
    sim_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    ${PROJECT_NAME}_controller_to_csv_main
    ${PROJECT_NAME}_csv_to_controller_main
    ${PROJECT_NAME}_keyboard_teleop
    ${PROJECT_NAME}_sim_hw_main
    ${PROJECT_NAME}_test_trajectory
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
