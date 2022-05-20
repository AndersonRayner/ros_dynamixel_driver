# dynamixel_ros_driver
A simple ROS driver for Dynamixel servos.
This driver controlled the servos via services so that feedback is returned when the servo has acknowledged the commands.
Tested Dynamixel servos
* XL330-M228

## Getting the Code
This code relies on the Dynamixel SDK, so you must clone the code recursively
```
https://github.com/AndersonRayner/dynamixel_ros_driver.git --recursive
```

## Interfacing
This code is designed to interface via the U2D2 adaptor (from Dynamixel) which creates a USB-to-Dynamixel bridge.

## Adding New Servos
All the commands are defined via the header file `dynamixel_driver/include/dynamixel_driver/control_table_addresses.h`.
For servos that use different control table addresses, simply change this file.
