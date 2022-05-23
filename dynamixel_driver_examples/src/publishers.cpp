
#include "dynamixel_driver_examples/drive_mode.h"

#include "std_msgs/Float32.h"


// Publishers
ros::Publisher presentVelocity_pub;
ros::Publisher presentPosition_pub;
ros::Publisher presentCurrent_pub;
ros::Publisher presentPwm_pub;

void init_publishers(ros::NodeHandle n)
{
 
  // Init all the publishers
  presentVelocity_pub = n.advertise<std_msgs::Float32>("/presentVelocity/value", 5);
  presentPosition_pub = n.advertise<std_msgs::Float32>("/presentPosition/value", 5);
  presentCurrent_pub  = n.advertise<std_msgs::Float32>("/presentCurrent/value", 5);
  presentPwm_pub      = n.advertise<std_msgs::Float32>("/presentPwm/value", 5);

  return;

}

void update_publishers() 
{
  const bool print_debugging = 0;
  std_msgs::Float32 msg;
  int32_t res;

  // Present velocity
  if (getPresentVelocity(&res))
  {
    msg.data = (float) res;
    presentVelocity_pub.publish(msg);
  } 

  // Present position
  if (getPresentPosition(&res))
  {
    msg.data = (float) res;
    presentPosition_pub.publish(msg);
  } 

  // Present current
  if (getPresentCurrent(&msg.data)) presentCurrent_pub.publish(msg);

  // Present PWM
  if (getPresentPwm(&msg.data)) presentPwm_pub.publish(msg);


  return;
  
}
