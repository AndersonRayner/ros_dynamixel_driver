################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_driver_examples)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    message_generation
    dynamixel_sdk
    dynamixel_driver
)


################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamixel_sdk
    dynamixel_driver
#  DEPENDS system_lib
)

## Libraries
add_library(${PROJECT_NAME}_libraries
  src/services.cpp
  src/publishers.cpp
)

## Includes
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Executables
# LED Flasher
add_executable(led_toggle src/led_toggle.cpp)
add_dependencies(led_toggle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(led_toggle 
  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}_libraries
)

# Sampler Test
add_executable(drive_mode src/drive_mode.cpp)
add_dependencies(drive_mode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_mode 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_libraries
)
