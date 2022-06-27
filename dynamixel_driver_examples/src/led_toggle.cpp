#include <ros/ros.h>

#include "dynamixel_msgs/SetLedEnable.h"

uint8_t id = 0x01;  // Target dynamixel ID (0xFE is broadcast)
                    //   Broadcast won't give feedback so return might fail

int main(int argc, char **argv)
{
  ros::init(argc, argv, "led_toggle");
  ros::NodeHandle n;
  ros::ServiceClient setLedEnableClient = n.serviceClient<dynamixel_msgs::SetLedEnable>("/ledEnable/set", true);

  ros::Rate loop_rate(1);

  int count = 0;

  while (ros::ok())
  {

    // Assemble the request packet
    dynamixel_msgs::SetLedEnable srv;
    srv.request.id = id;

    if (count % 2) 
    {
        srv.request.ledEnable = dynamixel_msgs::SetLedEnable::Request::LED_ON; 
    } else {
        srv.request.ledEnable = dynamixel_msgs::SetLedEnable::Request::LED_OFF;
    }

    // Call the service
    ROS_INFO("Changing LED State : %d", srv.request.ledEnable);

    if (setLedEnableClient.call(srv) == 0)
    {
      ROS_ERROR("Failed to call setLedEnable service");
    }

    // ROS stuff
    ros::spinOnce();
    loop_rate.sleep();

    ++count;

  }

  return 0;
}
