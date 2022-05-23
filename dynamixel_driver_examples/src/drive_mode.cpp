
#include "dynamixel_driver_examples/drive_mode.h"

uint8_t _servo_id = 0x01; // Target dynamixel ID (0xFE is broadcast)
                          //   Broadcast won't give feedback so return might fail

enum class DRIVE_MODE
{
  MODE_VELOCITY_CONTROL,
  MODE_CURRENT_CONTROL,
  MODE_EXTENDED_POSITION_CONTROL,
  MODE_POSITION_CONTROL,
};

DRIVE_MODE _drive_mode;

void start_extended_position_control()
{
  uint errors = 0;

  // Change the mode
  do
  {
    if (setTorqueEnable(dynamixel_driver::SetTorqueEnable::Request::TORQUE_DISABLE) != 0)                   errors++;
    if (setOperatingMode(dynamixel_driver::SetOperatingMode::Request::MODE_EXTENDED_POSITION_CONTROL) != 0) errors++;
    if (setTorqueEnable(dynamixel_driver::SetTorqueEnable::Request::TORQUE_ENABLE) != 0)                    errors++;

    if (setGoalPosition(2000) != 0) errors++;

    if (errors)
    {
      // We have errors, wait and try again
      ros::Duration(0.5).sleep();
      errors = 0;
    }
    else
    {
      // No errors, we can move onto the next stage
      _drive_mode = DRIVE_MODE::MODE_EXTENDED_POSITION_CONTROL;
      
      ROS_INFO("Switching to MODE_EXTENDED_POSITION_CONTROL");
    }
  } while (errors);

  return;
}
void start_position_control()
{
  uint errors = 0;

  do
  {
    if (setTorqueEnable(dynamixel_driver::SetTorqueEnable::Request::TORQUE_DISABLE) != 0)          errors++;
    if (setOperatingMode(dynamixel_driver::SetOperatingMode::Request::MODE_POSITION_CONTROL) != 0) errors++;
    if (setTorqueEnable(dynamixel_driver::SetTorqueEnable::Request::TORQUE_ENABLE) != 0)           errors++;

    if (setGoalPosition(0) != 0)
      errors++;

    if (errors)
    {
      // We have errors, wait and try again
      ros::Duration(0.5).sleep();
      errors = 0;
    }
    else
    {
      // No errors, we can move onto the next stage
      _drive_mode = DRIVE_MODE::MODE_POSITION_CONTROL;

      ROS_INFO("Switching to MODE_POSITION_CONTROL");
    }
  } while (errors);

  return;

}
void start_velocity_control()
{
  uint errors = 0;

  // Change the mode
  do
  {
    if (setTorqueEnable(dynamixel_driver::SetTorqueEnable::Request::TORQUE_DISABLE) != 0)         errors++;
    if (setOperatingMode(dynamixel_driver::SetOperatingMode::Request::MODE_VELOCITY_CONTROL) != 0) errors++;
    if (setTorqueEnable(dynamixel_driver::SetTorqueEnable::Request::TORQUE_ENABLE) != 0)          errors++;

    if (setGoalVelocity(-50) != 0) errors++;

    if (errors)
    {
      // We have errors, wait and try again
      ros::Duration(0.5).sleep();
      errors = 0;
    }
    else
    {
      // No errors, we can move onto the next stage
      _drive_mode = DRIVE_MODE::MODE_VELOCITY_CONTROL;
      
      ROS_INFO("Switching to MODE_VELOCITY_CONTROL");
    }
  } while (errors);

  return;
}
void start_current_control()
{
  uint errors = 0;

  // Change the mode
  do
  {
    if (setTorqueEnable(dynamixel_driver::SetTorqueEnable::Request::TORQUE_DISABLE) != 0)         errors++;
    if (setOperatingMode(dynamixel_driver::SetOperatingMode::Request::MODE_CURRENT_CONTROL) != 0) errors++;
    if (setTorqueEnable(dynamixel_driver::SetTorqueEnable::Request::TORQUE_ENABLE) != 0)          errors++;

    if (setGoalCurrent(50) != 0) errors++;

    if (errors)
    {
      // We have errors, wait and try again
      ros::Duration(0.5).sleep();
      errors = 0;
    }
    else
    {
      // No errors, we can move onto the next stage
      _drive_mode = DRIVE_MODE::MODE_CURRENT_CONTROL;
      
      ROS_INFO("Switching to MODE_CURRENT_CONTROL");
    }
  } while (errors);

  return;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "drive_mode");
  ros::NodeHandle n;

  init_publishers(n);
  init_services(n);

  ros::Rate loop_rate(10);

  int count = 0;
  _drive_mode = DRIVE_MODE::MODE_VELOCITY_CONTROL;

  // Set the state entry time in the past so the servo is automatically set up
  ros::Time t_state_entry = ros::Time::now() - ros::Duration(100.0);

  ROS_INFO("Running the Drive Mode Test Script");

  // Loop through the sample cycle
  while (ros::ok())
  {

    // State Machine
    switch (_drive_mode)
    {

    case (DRIVE_MODE::MODE_VELOCITY_CONTROL):
    {
      // Change the mode after a short amount of time
      if (ros::Time::now() - t_state_entry > ros::Duration(5.0))
      {
        start_current_control();
        t_state_entry = ros::Time::now();
      }

      break;
    }
    case (DRIVE_MODE::MODE_CURRENT_CONTROL):
    {
      // Change the mode after a short amount of time
      if (ros::Time::now() - t_state_entry > ros::Duration(5.0))
      {
        start_position_control();
        t_state_entry = ros::Time::now();
      }

      break;
    }
    case (DRIVE_MODE::MODE_POSITION_CONTROL):
    {

      // Change the mode after a short amount of time
      if (ros::Time::now() - t_state_entry > ros::Duration(5.0))
      {
        start_extended_position_control();
        t_state_entry = ros::Time::now();
      }

      break;
    }
    case (DRIVE_MODE::MODE_EXTENDED_POSITION_CONTROL):
    {

      // Change the mode after a short amount of time
      if (ros::Time::now() - t_state_entry > ros::Duration(5.0))
      {
        start_velocity_control();
        t_state_entry = ros::Time::now();

      }

      break;
    }

    }
    // Update the publishers
    update_publishers();

    // Do ROS Stuff
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
