
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
  //    The second argument 'true' is important for speed as this means ros doesn't have to 
  //    renegotiate the connection each time the service is called.
  getHomingOffsetClient    = n.serviceClient<dynamixel_msgs::GetHomingOffset>("/homingOffset/get", true);
  getGoalPositionClient    = n.serviceClient<dynamixel_msgs::GetGoalPosition>("/goalPosition/get", true);
  getMovingStatusClient    = n.serviceClient<dynamixel_msgs::GetMovingStatus>("/movingStatus/get", true);
  getPresentCurrentClient  = n.serviceClient<dynamixel_msgs::GetPresentCurrent>("/presentCurrent/get", true);
  getPresentPositionClient = n.serviceClient<dynamixel_msgs::GetPresentPosition>("/presentPosition/get", true);
  getPresentPwmClient      = n.serviceClient<dynamixel_msgs::GetPresentPwm>("/presentPwm/get", true);
  getPresentVelocityClient = n.serviceClient<dynamixel_msgs::GetPresentVelocity>("/presentVelocity/get", true);
  getTorqueEnableClient    = n.serviceClient<dynamixel_msgs::GetTorqueEnable>("/torqueEnable/get", true);
  getOperatingModeClient   = n.serviceClient<dynamixel_msgs::GetOperatingMode>("/operatingMode/get", true);
  
  setGoalCurrentClient = n.serviceClient<dynamixel_msgs::SetGoalCurrent>("/goalCurrent/set", true);
  setHomingOffsetClient = n.serviceClient<dynamixel_msgs::SetHomingOffset>("/homingOffset/set", true);
  setLedEnableClient = n.serviceClient<dynamixel_msgs::SetLedEnable>("/ledEnable/set", true);
  setTorqueEnableClient = n.serviceClient<dynamixel_msgs::SetTorqueEnable>("/torqueEnable/set", true);
  setGoalVelocityClient = n.serviceClient<dynamixel_msgs::SetGoalVelocity>("/goalVelocity/set", true);
  setGoalPositionClient = n.serviceClient<dynamixel_msgs::SetGoalPosition>("/goalPosition/set", true);
  setOperatingModeClient = n.serviceClient<dynamixel_msgs::SetOperatingMode>("/operatingMode/set", true);
  setProfileVelocityClient = n.serviceClient<dynamixel_msgs::SetProfileVelocity>("/profileVelocity/set", true);
  setVelocityLimitClient = n.serviceClient<dynamixel_msgs::SetVelocityLimit>("/velocityLimit/set", true);
  setVelocityPGainClient = n.serviceClient<dynamixel_msgs::SetVelocityPGain>("/velocityPGain/set", true);
  setVelocityIGainClient = n.serviceClient<dynamixel_msgs::SetVelocityIGain>("/velocityIGain/set", true);

  return;

}

int32_t getGoalPosition()
{
  dynamixel_msgs::GetGoalPosition srv;

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
  dynamixel_msgs::GetMovingStatus srv;

  // Set the dynamixel ID
  srv.request.id = _servo_id;

  if (getMovingStatusClient.call(srv))
  {

    //ROS_INFO("getGoalReached successful");

    if ( (srv.response.profileOngoing == dynamixel_msgs::GetMovingStatus::Response::PROFILE_COMPLETED) &&
         (srv.response.inPosition     == dynamixel_msgs::GetMovingStatus::Response::ARRIVED)            )
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
  dynamixel_msgs::GetPresentPwm srv;

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
  dynamixel_msgs::GetPresentVelocity srv;

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
  dynamixel_msgs::GetPresentCurrent srv;

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
  dynamixel_msgs::GetPresentPosition srv;

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
  dynamixel_msgs::GetTorqueEnable srv;

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
  dynamixel_msgs::GetOperatingMode srv;

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

  dynamixel_msgs::SetGoalCurrent srv;


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

  dynamixel_msgs::SetHomingOffset srv;


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

  dynamixel_msgs::SetTorqueEnable srv;


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
  dynamixel_msgs::SetOperatingMode srv;
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

  dynamixel_msgs::SetGoalVelocity srv;

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

  dynamixel_msgs::SetGoalPosition srv;

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
  dynamixel_msgs::SetLedEnable srv;
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
  dynamixel_msgs::SetVelocityLimit srv;
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
  dynamixel_msgs::SetProfileVelocity srv;
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
  dynamixel_msgs::SetVelocityPGain srv1;
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
  dynamixel_msgs::SetVelocityIGain srv2;
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