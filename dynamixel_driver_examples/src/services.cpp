
#include "dynamixel_driver_examples/drive_mode.h"

ros::ServiceClient getHomingOffsetClient;
ros::ServiceClient getGoalPositionClient;
ros::ServiceClient getMovingStatusClient;
ros::ServiceClient getPresentCurrentClient;
ros::ServiceClient getPresentPositionClient;
ros::ServiceClient getPresentPwmClient;
ros::ServiceClient getPresentVelocityClient;
ros::ServiceClient getTorqueEnableClient;
ros::ServiceClient getOperatingModeClient;

ros::ServiceClient setGoalCurrentClient;
ros::ServiceClient setHomingOffsetClient;
ros::ServiceClient setLedEnableClient;
ros::ServiceClient setGoalPositionClient;
ros::ServiceClient setGoalVelocityClient;
ros::ServiceClient setOperatingModeClient;
ros::ServiceClient setProfileVelocityClient;
ros::ServiceClient setTorqueEnableClient;
ros::ServiceClient setVelocityLimitClient;
ros::ServiceClient setVelocityPGainClient;
ros::ServiceClient setVelocityIGainClient;

void init_services(ros::NodeHandle n)
{
 
 // Add the torque service in here
  getHomingOffsetClient    = n.serviceClient<dynamixel_driver::GetHomingOffset>("/homingOffset/get", true);
  getGoalPositionClient    = n.serviceClient<dynamixel_driver::GetGoalPosition>("/goalPosition/get", true);
  getMovingStatusClient    = n.serviceClient<dynamixel_driver::GetMovingStatus>("/movingStatus/get", true);
  getPresentCurrentClient  = n.serviceClient<dynamixel_driver::GetPresentCurrent>("/presentCurrent/get", true);
  getPresentPositionClient = n.serviceClient<dynamixel_driver::GetPresentPosition>("/presentPosition/get", true);
  getPresentPwmClient      = n.serviceClient<dynamixel_driver::GetPresentPwm>("/presentPwm/get", true);
  getPresentVelocityClient = n.serviceClient<dynamixel_driver::GetPresentVelocity>("/presentVelocity/get", true);
  getTorqueEnableClient    = n.serviceClient<dynamixel_driver::GetTorqueEnable>("/torqueEnable/get", true);
  getOperatingModeClient   = n.serviceClient<dynamixel_driver::GetOperatingMode>("/operatingMode/get", true);
  
  setGoalCurrentClient = n.serviceClient<dynamixel_driver::SetGoalCurrent>("/goalCurrent/set", true);
  setHomingOffsetClient = n.serviceClient<dynamixel_driver::SetHomingOffset>("/homingOffset/set", true);
  setLedEnableClient = n.serviceClient<dynamixel_driver::SetLedEnable>("/ledEnable/set", true);
  setTorqueEnableClient = n.serviceClient<dynamixel_driver::SetTorqueEnable>("/torqueEnable/set", true);
  setGoalVelocityClient = n.serviceClient<dynamixel_driver::SetGoalVelocity>("/goalVelocity/set", true);
  setGoalPositionClient = n.serviceClient<dynamixel_driver::SetGoalPosition>("/goalPosition/set", true);
  setOperatingModeClient = n.serviceClient<dynamixel_driver::SetOperatingMode>("/operatingMode/set", true);
  setProfileVelocityClient = n.serviceClient<dynamixel_driver::SetProfileVelocity>("/profileVelocity/set", true);
  setVelocityLimitClient = n.serviceClient<dynamixel_driver::SetVelocityLimit>("/velocityLimit/set", true);
  setVelocityPGainClient = n.serviceClient<dynamixel_driver::SetVelocityPGain>("/velocityPGain/set", true);
  setVelocityIGainClient = n.serviceClient<dynamixel_driver::SetVelocityIGain>("/velocityIGain/set", true);

  return;

}

int32_t getGoalPosition()
{
  dynamixel_driver::GetGoalPosition srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getGoalPositionClient.call(srv))
  {
    return (srv.response.position);
  }
  else
  {
    ROS_ERROR("Failed to call goalPosition service");
    return (-1); // bad lol
  }

  return (srv.response.result);

}

int32_t getGoalReached(uint8_t *goalReached)
{
  dynamixel_driver::GetMovingStatus srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getMovingStatusClient.call(srv))
  {

    //ROS_INFO("getGoalReached successful");

    if ( (srv.response.profileOngoing == dynamixel_driver::GetMovingStatus::Response::PROFILE_COMPLETED) &&
         (srv.response.inPosition     == dynamixel_driver::GetMovingStatus::Response::ARRIVED)            )
    {
      *goalReached = 1;
    }
    else
    {
      *goalReached = 0;
    }

  }
  else
  {
    ROS_ERROR("Failed to call movingStatus service");
  }

  return (srv.response.result);
}

int32_t getPresentPwm(float *presentPwm)
{
  dynamixel_driver::GetPresentPwm srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getPresentPwmClient.call(srv))
  {
    const float scale_factor = 0.113f;   // Scale factor from datasheet
    *presentPwm = (float) srv.response.pwm * scale_factor; // value is in [ % ]
  }
  else
  {
    ROS_ERROR("Failed to call presentPwm service");
  }

  return (srv.response.result);

}

int32_t getPresentVelocity(int32_t *presentVelocity)
{
  dynamixel_driver::GetPresentVelocity srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getPresentVelocityClient.call(srv))
  {
    *presentVelocity = srv.response.velocity;
  }
  else
  {
    ROS_ERROR("Failed to call getPresentVelocity service");
  }

  return (srv.response.result);

}


int32_t getPresentCurrent(float *presentCurrent)
{
  dynamixel_driver::GetPresentCurrent srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getPresentCurrentClient.call(srv))
  {
    *presentCurrent = (float) srv.response.current / 1000.0f; // value is in [ mA ]
  }
  else
  {
    ROS_ERROR("Failed to call getPresentCurrent service");
  }

  return (srv.response.result);

}

int32_t getPresentPosition(int32_t *presentPosition)
{
  dynamixel_driver::GetPresentPosition srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getPresentPositionClient.call(srv))
  {
    *presentPosition = srv.response.position;
  }
  else
  {
    ROS_ERROR("Failed to call getPresentPosition service");
  }

  return (srv.response.result);

}

int32_t getTorqueEnabled(uint8_t *torqueEnable)
{
  dynamixel_driver::GetTorqueEnable srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getTorqueEnableClient.call(srv))
  {
    *torqueEnable = srv.response.enable;
  }
  else
  {
    ROS_ERROR("Failed to call getTorqueEnabled service");
  }
  
  return (srv.response.result);

}

int32_t getOperatingMode(uint8_t *operatingMode)
{
  dynamixel_driver::GetOperatingMode srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getOperatingModeClient.call(srv))
  {
    *operatingMode = srv.response.operatingMode;
  }
  else
  {
    ROS_ERROR("Failed to call operatingMode service");
  }

  return (srv.response.result);

}

// SET FUNCTIONS //

int32_t setGoalCurrent(uint16_t goalCurrent)
{

  dynamixel_driver::SetGoalCurrent srv;


  // Set the dynamixel ID
  srv.request.id = _servo_id;
  srv.request.current = goalCurrent;

  if (setGoalCurrentClient.call(srv))
  {
    //ROS_INFO("setGoalCurrent successful");
  }
  else
  {
    ROS_ERROR("Failed to call setGoalCurrent service");
  }

  return (srv.response.result);

}

int32_t setHomingOffset(int32_t homingOffset)
{

  dynamixel_driver::SetHomingOffset srv;


  // Set the dynamixel ID
  srv.request.id = _servo_id;
  srv.request.homingOffset = homingOffset;

  if (setHomingOffsetClient.call(srv))
  {
    //ROS_INFO("setTorqueEnable successful");
  }
  else
  {
    ROS_ERROR("Failed to call setHomingOffsetClient service");
  }

  return (srv.response.result);

}

int32_t setTorqueEnable(uint8_t enable)
{

  dynamixel_driver::SetTorqueEnable srv;


  // Set the dynamixel ID
  srv.request.id = _servo_id;
  srv.request.torqueEnable = enable;

  if (setTorqueEnableClient.call(srv))
  {
    //ROS_INFO("setTorqueEnable successful");
  }
  else
  {
    ROS_ERROR("Failed to call setTorqueEnable service");
  }

  return (srv.response.result);

}

int32_t setOperatingMode(uint8_t mode)
{

  // Change the mode
  dynamixel_driver::SetOperatingMode srv;
  srv.request.id = _servo_id;
  srv.request.operatingMode = mode;

  if (setOperatingModeClient.call(srv))
  {
    //ROS_INFO("setOperatingMode successful");
  }
  else
  {
    ROS_ERROR("Failed to call setOperatingMode service");
  }

  // All done
  return (srv.response.result);
}

int32_t setGoalVelocity(int32_t speed)
{

  dynamixel_driver::SetGoalVelocity srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;
  srv.request.velocity = speed;

  if (setGoalVelocityClient.call(srv))
  {
    //ROS_INFO("setGoalVelocity successful");
  }
  else
  {
    ROS_ERROR("Failed to call setGoalVelocity service");
  }

  return (srv.response.result);

}

int32_t setGoalPosition(int32_t position)
{

  dynamixel_driver::SetGoalPosition srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;
  srv.request.position = position;

  if (setGoalPositionClient.call(srv))
  {
    //ROS_INFO("setGoalPosition successful");
  }
  else
  {
    ROS_ERROR("Failed to call setGoalPosition service");
  }

  // All done
  return (srv.response.result);

}

int32_t setLED(uint8_t enable)
{
  dynamixel_driver::SetLedEnable srv;
  srv.request.id = _servo_id;
  srv.request.ledEnable = enable;

  if (setLedEnableClient.call(srv))
  {
    //ROS_INFO("setLedEnableClient successful");
  }
  else
  {
    ROS_ERROR("Failed to call setLedEnableClient service");
  }

  // All done
  return (srv.response.result);

}

int32_t setServoVelocityLimit(uint32_t velocityLimit)
{
  dynamixel_driver::SetVelocityLimit srv;
  srv.request.id = _servo_id;
  srv.request.velocityLimit = velocityLimit;

  if (setVelocityLimitClient.call(srv))
  {
    //ROS_INFO("setVelocityLimitClient successful");
  }
  else
  {
    ROS_ERROR("Failed to call setVelocityLimitClient service");
  }

  // All done
  return (srv.response.result);

}

int32_t setProfileVelocity(uint32_t profileVelocity)
{
  dynamixel_driver::SetProfileVelocity srv;
  srv.request.id = _servo_id;
  srv.request.profileVelocity = profileVelocity;

  if (setProfileVelocityClient.call(srv))
  {
    //ROS_INFO("setProfileVelocityClient successful");
  }
  else
  {
    ROS_ERROR("Failed to call setProfileVelocityClient service");
  }

  // All done
  return (srv.response.result);

}

int32_t setServoVelocityPIGain(uint16_t P, uint16_t I)
{
  // Set the P Gain
  dynamixel_driver::SetVelocityPGain srv1;
  srv1.request.id = _servo_id;
  srv1.request.velocityPGain = P;

  if (setVelocityPGainClient.call(srv1))
  {
    //ROS_INFO("setVelocityPGainClient successful");
  }
  else
  {
    ROS_ERROR("Failed to call setVelocityPGainClient service");
    return (srv1.response.result);
  }

  // Set the I Gain
  dynamixel_driver::SetVelocityIGain srv2;
  srv2.request.id = _servo_id;
  srv2.request.velocityIGain = I;

  if (setVelocityIGainClient.call(srv2))
  {
    //ROS_INFO("setVelocityIGainClient successful");
  }
  else
  {
    ROS_ERROR("Failed to call setVelocityIGainClient service");
    return (srv2.response.result);
  }

  // Return the response from call 2
  return (srv2.response.result);

}