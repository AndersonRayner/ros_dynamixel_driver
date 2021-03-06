
#include "dynamixel_driver/dynamixel_driver.h"

// Get Includes
#include "dynamixel_msgs/GetGoalPosition.h"
#include "dynamixel_msgs/GetHomingOffset.h"
#include "dynamixel_msgs/GetModelInfo.h"
#include "dynamixel_msgs/GetModelNumber.h"
#include "dynamixel_msgs/GetMoving.h"
#include "dynamixel_msgs/GetMovingStatus.h"
#include "dynamixel_msgs/GetOperatingMode.h"
#include "dynamixel_msgs/GetPresentCurrent.h"
#include "dynamixel_msgs/GetPresentInputVoltage.h"
#include "dynamixel_msgs/GetPresentPosition.h"
#include "dynamixel_msgs/GetPresentPwm.h"
#include "dynamixel_msgs/GetPresentTemperature.h"
#include "dynamixel_msgs/GetPresentVelocity.h"
#include "dynamixel_msgs/GetRealtimeTick.h"
#include "dynamixel_msgs/GetTorqueEnable.h"

// Set Includes
#include "dynamixel_msgs/SetGoalCurrent.h"
#include "dynamixel_msgs/SetGoalPosition.h"
#include "dynamixel_msgs/SetGoalVelocity.h"
#include "dynamixel_msgs/SetHomingOffset.h"
#include "dynamixel_msgs/SetLedEnable.h"
#include "dynamixel_msgs/SetOperatingMode.h"
#include "dynamixel_msgs/SetProfileVelocity.h"
#include "dynamixel_msgs/SetTorqueEnable.h"
#include "dynamixel_msgs/SetVelocityLimit.h"
#include "dynamixel_msgs/SetVelocityPGain.h"
#include "dynamixel_msgs/SetVelocityIGain.h"

// Get Services
ros::ServiceServer getGoalPosition_srv;
ros::ServiceServer getHomingOffset_srv;
ros::ServiceServer getModelInfo_srv;
ros::ServiceServer getModelNumber_srv;
ros::ServiceServer getMoving_srv;
ros::ServiceServer getMovingStatus_srv;
ros::ServiceServer getOperatingMode_srv;
ros::ServiceServer getPresentCurrent_srv;
ros::ServiceServer getPresentInputVoltage_srv;
ros::ServiceServer getPresentPosition_srv;
ros::ServiceServer getPresentPwm_srv;
ros::ServiceServer getPresentTemperature_srv;
ros::ServiceServer getPresentVelocity_srv;
ros::ServiceServer getRealtimeTick_srv;
ros::ServiceServer getTorqueEnable_srv;

ros::ServiceServer setGoalCurrent_srv;
ros::ServiceServer setGoalPosition_srv;
ros::ServiceServer setGoalVelocity_srv;
ros::ServiceServer setHomingOffset_srv;
ros::ServiceServer setLedEnable_srv;
ros::ServiceServer setOperatingMode_srv;
ros::ServiceServer setProfileVelocity_srv;
ros::ServiceServer setTorqueEnable_srv;
ros::ServiceServer setVelocityLimit_srv;
ros::ServiceServer setVelocityPGain_srv;
ros::ServiceServer setVelocityIGain_srv;

// GET CALLBACKS //

bool getGoalPositionCallback(
    dynamixel_msgs::GetGoalPosition::Request &req,
    dynamixel_msgs::GetGoalPosition::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_GOAL_POSITION, (uint32_t *)&res.position, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getPosition : [ID:%d] -> [GOAL POSITION:%d]", req.id, res.position);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get goal position! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getHomingOffsetCallback(
    dynamixel_msgs::GetHomingOffset::Request &req,
    dynamixel_msgs::GetHomingOffset::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_HOMING_OFFSET, (uint32_t *)&res.homingOffset, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getHomingOffset : [ID:%d] -> [HOMING OFFSET:%d]", req.id, res.homingOffset);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get homing offset! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getModelInfoCallback(
    dynamixel_msgs::GetModelInfo::Request &req,
    dynamixel_msgs::GetModelInfo::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_MODEL_INFO, (uint32_t *)&res.modelInfo, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    //ROS_INFO("getModelInfo : [ID:%d] -> [MODEL INFO:%d]", req.id, res.modelInfo);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get model info! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getModelNumberCallback(
    dynamixel_msgs::GetModelNumber::Request &req,
    dynamixel_msgs::GetModelNumber::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_MODEL_NUMBER, (uint16_t *)&res.modelNumber, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getModelNumber : [ID:%d] -> [MODEL NUMBER:%d]", req.id, res.modelNumber);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get model number! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getMovingCallback(
    dynamixel_msgs::GetMoving::Request &req,
    dynamixel_msgs::GetMoving::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read1ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_MOVING, (uint8_t *)&res.moving, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getMoving : [ID:%d] -> [MOVING:%d]", req.id, res.moving);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get moving! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getMovingStatusCallback(
    dynamixel_msgs::GetMovingStatus::Request &req,
    dynamixel_msgs::GetMovingStatus::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  uint8_t movingStatus = 0;

  dxl_comm_result = packetHandler->read1ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_MOVING_STATUS, (uint8_t *)&movingStatus, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getArrived : [ID:%d] -> [MOVING_STATUS:%d]", req.id, movingStatus);

    // Extract the bits out of the byte
    res.velocityProfile = (movingStatus >> 4) & 0x03;
    res.followingError = (movingStatus >> 3) & 0x01;
    res.profileOngoing = (movingStatus >> 1) & 0x01;
    res.inPosition = (movingStatus >> 0) & 0x01;
    return true;
  }
  else
  {
    ROS_INFO("Failed to get moving status! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getOperatingModeCallback(
    dynamixel_msgs::GetOperatingMode::Request &req,
    dynamixel_msgs::GetOperatingMode::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read1ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_OPERATING_MODE, (uint8_t *)&res.operatingMode, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getMode : [ID:%d] -> [OPERATING MODE:%d]", req.id, res.operatingMode);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getPresentCurrentCallback(
    dynamixel_msgs::GetPresentCurrent::Request &req,
    dynamixel_msgs::GetPresentCurrent::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_PRESENT_CURRENT, (uint16_t *)&res.current, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getPresentCurrent : [ID:%d] -> [CURRENT:%d]", req.id, res.current);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get present current! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getPresentInputVoltageCallback(
    dynamixel_msgs::GetPresentInputVoltage::Request &req,
    dynamixel_msgs::GetPresentInputVoltage::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_PRESENT_INPUT_VOLTAGE, (uint16_t *)&res.voltage, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getInputVoltage : [ID:%d] -> [INPUT VOLTAGE:%d]", req.id, res.voltage);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get input voltage! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getPresentPositionCallback(
    dynamixel_msgs::GetPresentPosition::Request &req,
    dynamixel_msgs::GetPresentPosition::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&res.position, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, res.position);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getPresentPwmCallback(
    dynamixel_msgs::GetPresentPwm::Request &req,
    dynamixel_msgs::GetPresentPwm::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_PRESENT_PWM, (uint16_t *)&res.pwm, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getPresentPwm : [ID:%d] -> [PWM:%d]", req.id, res.pwm);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get present PWM! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getPresentTemperatureCallback(
    dynamixel_msgs::GetPresentTemperature::Request &req,
    dynamixel_msgs::GetPresentTemperature::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t val = 0;

  dxl_comm_result = packetHandler->read1ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_PRESENT_TEMPERATURE, (uint8_t *)&res.temperature, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getTemperature : [ID:%d] -> [TEMPERATURE:%d]", req.id, val);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get temperature! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getPresentVelocityCallback(
    dynamixel_msgs::GetPresentVelocity::Request &req,
    dynamixel_msgs::GetPresentVelocity::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_PRESENT_VELOCITY, (uint32_t *)&res.velocity, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getVelocity : [ID:%d] -> [VELOCITY:%d]", req.id, res.velocity);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get velocity! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getRealtimeTickCallback(
    dynamixel_msgs::GetRealtimeTick::Request &req,
    dynamixel_msgs::GetRealtimeTick::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_PRESENT_VELOCITY, (uint16_t *)&res.realtimeTick, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getRealtimeTick : [ID:%d] -> [REALTIME TICK:%d]", req.id, res.realtimeTick);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get realtime tick! Result: %d", dxl_comm_result);
    return false;
  }
}

bool getTorqueEnableCallback(
    dynamixel_msgs::GetTorqueEnable::Request &req,
    dynamixel_msgs::GetTorqueEnable::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read1ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_TORQUE_ENABLE, (uint8_t *)&res.enable, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getTorqueEnabled : [ID:%d] -> [ENABLED:%d]", req.id, val);
    return true;
  }
  else
  {
    ROS_INFO("Failed to get temperature! Result: %d", dxl_comm_result);
    return false;
  }
}

// SET CALLBACKS //

bool setGoalCurrentCallback(
    dynamixel_msgs::SetGoalCurrent::Request &req,
    dynamixel_msgs::SetGoalCurrent::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_GOAL_CURRENT, (uint16_t)req.current, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setGoalCurrent : [ID:%d] -> [CURRENT:%d]", req.id, req.current);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set goal current! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setGoalPositionCallback(
    dynamixel_msgs::SetGoalPosition::Request &req,
    dynamixel_msgs::SetGoalPosition::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_GOAL_POSITION, (uint32_t)req.position, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setGoalPosition : [ID:%d] -> [POSITION:%d]", req.id, req.position);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set goal position mode! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setGoalVelocityCallback(
    dynamixel_msgs::SetGoalVelocity::Request &req,
    dynamixel_msgs::SetGoalVelocity::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_GOAL_VELOCITY, (uint32_t)req.velocity, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setGoalVelocity : [ID:%d] -> [VELOCITY:%d]", req.id, req.velocity);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set goal velocity! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setHomingOffsetCallback(
    dynamixel_msgs::SetHomingOffset::Request &req,
    dynamixel_msgs::SetHomingOffset::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_HOMING_OFFSET, (uint32_t)req.homingOffset, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setHomingOffset : [ID:%d] -> [HOMING OFFSET:%d]", req.id, req.homingOffset);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set homing offset! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setLedEnableCallback(
    dynamixel_msgs::SetLedEnable::Request &req,
    dynamixel_msgs::SetLedEnable::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_LED_ENABLE, req.ledEnable, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setLedEnable : [ID:%d] -> [LED ENABLE:%d]", req.id, req.ledEnable);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set led enable! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setOperatingModeCallback(
    dynamixel_msgs::SetOperatingMode::Request &req,
    dynamixel_msgs::SetOperatingMode::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_OPERATING_MODE, req.operatingMode, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setOperatingMode : [ID:%d] -> [OPERATING MODE:%d]", req.id, req.operatingMode);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set operating mode! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setProfileVelocityCallback(
    dynamixel_msgs::SetProfileVelocity::Request &req,
    dynamixel_msgs::SetProfileVelocity::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_PROFILE_VELOCITY, req.profileVelocity, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("profileVelocity : [ID:%d] -> [PROFILE VELOCITY:%d]", req.id, req.profileVelocity);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set operating mode! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setTorqueEnableCallback(
    dynamixel_msgs::SetTorqueEnable::Request &req,
    dynamixel_msgs::SetTorqueEnable::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_TORQUE_ENABLE, req.torqueEnable, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setTorqueEnable : [ID:%d] -> [TORQUE ENABLE:%d]", req.id, req.torqueEnable);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set torque enable! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setVelocityLimitCallback(
    dynamixel_msgs::SetVelocityLimit::Request &req,
    dynamixel_msgs::SetVelocityLimit::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_VELOCITY_LIMIT, req.velocityLimit, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setVelocityLimit : [ID:%d] -> [VELOCITY LIMIT:%d]", req.id, req.velocityLimit);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set velocity limit! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setVelocityPGainCallback(
    dynamixel_msgs::SetVelocityPGain::Request &req,
    dynamixel_msgs::SetVelocityPGain::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_VELOCITY_LIMIT, req.velocityPGain, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setVelocityPGain : [ID:%d] -> [VELOCITY P GAIN:%d]", req.id, req.velocityPGain);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set velocity P gain! Result: %d", dxl_comm_result);
    return (false);
  }
}

bool setVelocityIGainCallback(
    dynamixel_msgs::SetVelocityIGain::Request &req,
    dynamixel_msgs::SetVelocityIGain::Response &res)
{

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, (uint8_t)req.id, ADDR_VELOCITY_LIMIT, req.velocityIGain, &dxl_error);

  res.result = dxl_comm_result;
  sleep_comms();

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setVelocityIGain : [ID:%d] -> [VELOCITY I GAIN:%d]", req.id, req.velocityIGain);
    return (true);
  }
  else
  {
    ROS_INFO("Failed to set velocity I gain! Result: %d", dxl_comm_result);
    return (false);
  }
}


// INITIALISE SERVICES //

void init_services(ros::NodeHandle n)
{

  // Advertise get services
  getGoalPosition_srv = n.advertiseService("/goalPosition/get", getGoalPositionCallback);
  getHomingOffset_srv = n.advertiseService("/homingOffset/get", getHomingOffsetCallback);
  getModelInfo_srv = n.advertiseService("/modelInfo/get", getModelInfoCallback);
  getModelNumber_srv = n.advertiseService("/modelNumber/get", getModelNumberCallback);
  getMoving_srv = n.advertiseService("/moving/get", getMovingCallback);
  getMovingStatus_srv = n.advertiseService("/movingStatus/get", getMovingStatusCallback);
  getOperatingMode_srv = n.advertiseService("/operatingMode/get", getOperatingModeCallback);
  getPresentCurrent_srv = n.advertiseService("/presentCurrent/get", getPresentCurrentCallback);
  getPresentInputVoltage_srv = n.advertiseService("/presentInputVoltage/get", getPresentInputVoltageCallback);
  getPresentPosition_srv = n.advertiseService("/presentPosition/get", getPresentPositionCallback);
  getPresentPwm_srv = n.advertiseService("/presentPwm/get", getPresentPwmCallback);
  getPresentTemperature_srv = n.advertiseService("/presentTemperature/get", getPresentTemperatureCallback);
  getPresentVelocity_srv = n.advertiseService("/presentVelocity/get", getPresentVelocityCallback);
  getRealtimeTick_srv = n.advertiseService("/realtimeTick/get", getRealtimeTickCallback);
  getTorqueEnable_srv = n.advertiseService("/torqueEnable/get", getTorqueEnableCallback);

  // Advertise set services
  setGoalCurrent_srv = n.advertiseService("/goalCurrent/set", setGoalCurrentCallback);
  setGoalPosition_srv = n.advertiseService("/goalPosition/set", setGoalPositionCallback);
  setGoalVelocity_srv = n.advertiseService("/goalVelocity/set", setGoalVelocityCallback);
  setHomingOffset_srv = n.advertiseService("/homingOffset/set", setHomingOffsetCallback);
  setLedEnable_srv = n.advertiseService("/ledEnable/set", setLedEnableCallback);
  setOperatingMode_srv = n.advertiseService("/operatingMode/set", setOperatingModeCallback);
  setProfileVelocity_srv = n.advertiseService("/profileVelocity/set", setProfileVelocityCallback);
  setTorqueEnable_srv = n.advertiseService("/torqueEnable/set", setTorqueEnableCallback);
  setVelocityLimit_srv = n.advertiseService("/velocityLimit/set", setVelocityLimitCallback);
  setVelocityPGain_srv = n.advertiseService("/velocityPGain/set", setVelocityPGainCallback);
  setVelocityIGain_srv = n.advertiseService("/velocityIGain/set", setVelocityIGainCallback);

  // Services initialised
  return;
}