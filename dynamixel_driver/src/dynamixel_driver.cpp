
#include "dynamixel_driver/dynamixel_driver.h"

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600                  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyDynamixel" // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

int main(int argc, char **argv)
{

    // Open the port and set the baud rates
    if (!portHandler->openPort())
    {
        ROS_ERROR("Failed to open the port %s!",DEVICE_NAME);
        return -1;
    }

    if (!portHandler->setBaudRate(BAUDRATE))
    {
        ROS_ERROR("Failed to set the baudrate!");
        return -1;
    }

    // Init the ROS node
    ros::init(argc, argv, "dynamixel_driver_node");
    
    ros::NodeHandle n;

    // Init subscribers, publishers, services...
    init_subscribers(n);
    init_services(n);

    // Update user
    ROS_INFO("Dynamixel driver initialised on port %s::%d",DEVICE_NAME,BAUDRATE);

    // Spin forever, everything is callback based
    ros::spin();

    // Close port if we get to here before exiting
    portHandler->closePort();

    return 0;
}

void sleep_comms()
{
    // Sleep the comms to stop over-stressing the Dynamixels

    ros::Duration(0.005).sleep();
    
    return;
}