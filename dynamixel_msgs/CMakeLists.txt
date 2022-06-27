################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_msgs)

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
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

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
#  DEPENDS system_lib
)


