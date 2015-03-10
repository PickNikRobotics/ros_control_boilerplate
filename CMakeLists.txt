cmake_minimum_required(VERSION 2.8.3)
project(ros_control_boilerplate)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#add_subdirectory(myrobot_robot)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myrobot_ros_control
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

## Declare a cpp library
add_library(myrobot_ros_control src/myrobot_hardware_interface.cpp)
target_link_libraries(myrobot_ros_control
    ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(myrobot_node src/myrobot_node.cpp)
target_link_libraries(myrobot_node
    myrobot_ros_control
    ${catkin_LIBRARIES}
)

