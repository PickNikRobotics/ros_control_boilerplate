cmake_minimum_required(VERSION 2.8.3)
project(rrbot_control_boilerplate)

# C++ 11
add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
  ros_control_boilerplate
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrbot_hw_interface
  CATKIN_DEPENDS ros_control_boilerplate
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# RRBot Hardware Interface
add_library(rrbot_hw_interface
  src/rrbot_hw_interface.cpp
)
target_link_libraries(rrbot_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(rrbot_hw_main src/rrbot_hw_main.cpp)
target_link_libraries(rrbot_hw_main
  rrbot_hw_interface
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    rrbot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    rrbot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
