################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_driver)

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
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
    SetGoalPositionMsg.msg
    SetGoalVelocityMsg.msg
    #SetHomingOffsetMsg.msg
    #SetLedEnableMsg.msg
    #SetOperatingModeMsg.msg
    #SetTorqueEnableMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES

    GetGoalPosition.srv
    GetHomingOffset.srv
    GetModelInfo.srv
    GetModelNumber.srv
    GetMoving.srv
    GetMovingStatus.srv
    GetOperatingMode.srv
    GetPresentCurrent.srv
    GetPresentInputVoltage.srv
    GetPresentPosition.srv
    GetPresentPwm.srv
    GetPresentTemperature.srv
    GetPresentVelocity.srv
    GetRealtimeTick.srv
    GetTorqueEnable.srv
    
    SetGoalCurrent.srv
    SetGoalPosition.srv
    SetGoalVelocity.srv
    SetHomingOffset.srv
    SetLedEnable.srv
    SetOperatingMode.srv
    SetProfileVelocity.srv
    SetTorqueEnable.srv
    SetVelocityIGain.srv
    SetVelocityLimit.srv
    SetVelocityPGain.srv

)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamixel_sdk
#  DEPENDS system_lib
)

# Libraries
add_library(${PROJECT_NAME}_libraries
  src/services.cpp
  src/subscribers.cpp
)

################################################################################
# Build
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(dynamixel_driver src/dynamixel_driver.cpp)
add_dependencies(dynamixel_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_driver 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_libraries
)

################################################################################
# Install
################################################################################
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS dynamixel_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
