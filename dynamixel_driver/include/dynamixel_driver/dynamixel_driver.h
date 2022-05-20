
#pragma once

#include <ros/ros.h>

#include "std_msgs/String.h"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "dynamixel_driver/control_table_addresses.h"

// Variables
void sleep_comms(void); // Sleep the comms for the dynamixels

// Functions
void init_subscribers(ros::NodeHandle n);
void init_services(ros::NodeHandle n);

// Variables
extern dynamixel::PortHandler *  portHandler;
extern dynamixel::PacketHandler *  packetHandler;
