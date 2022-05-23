# dynamixel_ros_driver
A simple ROS driver for Dynamixel servos.
This driver controlled the servos via services so that feedback is returned when the servo has acknowledged the commands.
Tested Dynamixel servos
* XL330-M228

## Getting the Code
This code relies on the Dynamixel SDK, so you must clone the code recursively
```
git clone https://github.com/AndersonRayner/dynamixel_ros_driver.git --recursive
```

Given this is probably part of a larger project, you can also add it as a submodule in your catkin workspace
```
git submodule add https://github.com/AndersonRayner/dynamixel_ros_driver.git
```
Just remember to `--init` and `--update` submodules after adding it.

## Interfacing
### Hardware
This code is designed to interface via the U2D2 adaptor (from Dynamixel) which creates a USB-to-Dynamixel bridge.
You'll have to modify `dynamixel_driver/src/dynamixel_driver.cpp` to match your configuration, specifically the `BAUDRATE` and `DEVICE_NAME` variables.

### Software
Everything works via services so that you can get feedback as to if the servo has acknowledged the commands.
To increase the rate at which you can call the services, you need to subscribe to them using a persistent connection.
For example,
```
ros::NodeHandle n;
ros::ServiceClient setOperatingModeClient = n.serviceClient<dynamixel_driver::SetOperatingMode>("/operatingMode/set", true);
```

## Adding to the Code
### Adding New Servo Types
All the commands are defined via the header file `dynamixel_driver/include/dynamixel_driver/control_table_addresses.h`.
For servos that use different control table addresses, simply change this file.

### Issues and PRs
PRs are welcome of course!
There are some missing functions that are yet to be coded in, so if you need these, you can easily follow existing code to add them in.
