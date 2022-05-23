
#include "dynamixel_driver/dynamixel_driver.h"

dynamixel::PortHandler *portHandler;// = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
dynamixel::PacketHandler *packetHandler;// = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

int main(int argc, char **argv)
{

    // Init the ROS node
    ros::init(argc, argv, "dynamixel_driver_node");
    ros::NodeHandle nh("~");

    // Get the port parameters
    std::string port_name;
    int baud_rate;
    float protocol_version;

    nh.param<std::string>("port_name", port_name, std::string("/dev/ttyUSB0"));
    nh.param("baud_rate", baud_rate, 115200);
    nh.param("protocol_version", protocol_version, 2.0f);

    ROS_INFO("Attempting to open %s::%d, protocol %.1f",port_name.c_str(),baud_rate,protocol_version);

    // Open the port and set the baud rates
    portHandler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

    if (!portHandler->openPort())
    {
        ROS_ERROR("Failed to open the port %s!",port_name.c_str());
        return -1;
    }

    if (!portHandler->setBaudRate(baud_rate))
    {
        ROS_ERROR("Failed to set the baudrate!");
        return -1;
    }

    // Init subscribers, publishers, services...
    init_subscribers(nh);
    init_services(nh);

    // Update user
    ROS_INFO("Dynamixel driver initialised on port %s::%d",port_name.c_str(),baud_rate);

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