
#include "dynamixel_driver/dynamixel_driver.h"

//#include "dynamixel_driver/SetGoalPosition.h"
//#include "dynamixel_driver/SetTorqueEnable.h"
//#include "dynamixel_driver/SetLedEnable.h"
//#include "dynamixel_driver/SetOperatingMode.h"
//#include "dynamixel_driver/SetGoalVelocity.h"
//#include "dynamixel_driver/SetHomingOffset.h"

//ros::Subscriber set_goalPosition_sub;
//ros::Subscriber set_goalVelocity_sub;
//ros::Subscriber set_torque_sub;
//ros::Subscriber set_led_sub;
//ros::Subscriber set_operatingMode_sub;
//ros::Subscriber set_homingOffset_sub;

/*
void setPositionCallback(const dynamixel_driver::SetGoalPosition::ConstPtr &msg)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    int32_t  pos = msg->position;
    pos = 0;
    uint32_t unsigned_pos = (uint32_t) pos;

    // Write Goal Position (length : 4 bytes), with conversion from int32_t to uin32_t
    // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, (uint32_t) msg->position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    } 
    else
    {
        ROS_INFO("Set goal postion to %d",msg->position);
    }

    

    sleep_comms();
    return;
}
*/

/*
void setHomingOffsetCallback(const dynamixel_driver::SetHomingOffset::ConstPtr &msg)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler, (uint8_t)msg->id, ADDR_HOMING_OFFSET, (uint32_t)msg->homingOffset, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set homing offset! Result: %d", dxl_comm_result);
    }

    sleep_comms();
    return;
}
*/

/*
void setTorqueEnableCallback(const dynamixel_driver::SetTorqueEnable::ConstPtr &msg)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, (uint8_t)msg->id, ADDR_TORQUE_ENABLE, msg->torqueEnable, &dxl_error); //  the 1 here is the turn on
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to enable torque for Dynamixel ID %d", msg->id);
    }
    else
    {
        ROS_INFO("Torque value %d", msg->torqueEnable);
    }

    sleep_comms();
    return;
}
*/

/*
void setLedCallback(const dynamixel_driver::SetLedEnable::ConstPtr &msg)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, (uint8_t)msg->id, ADDR_LED_ENABLE, msg->ledEnable, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set LED for Dynamixel ID %d", msg->id);
    }

    sleep_comms();
    return;
}
*/

/*
void setOperatingModeCallback(const dynamixel_driver::SetOperatingMode::ConstPtr &msg)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, (uint8_t)msg->id, ADDR_OPERATING_MODE, msg->mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set operating mode for Dynamixel ID %d", msg->id);
    }

    sleep_comms();
    return;
}
*/

/*
void setGoalVelocityCallback(const dynamixel_driver::SetGoalVelocity::ConstPtr &msg)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler, (uint8_t)msg->id, ADDR_GOAL_VELOCITY, (uint32_t)msg->velocity, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set goal velocity for Dynamixel ID %d", msg->id);
    }

    sleep_comms();
    return;
}
*/

void init_subscribers(ros::NodeHandle n)
{
    //set_goalPosition_sub = n.subscribe("position/set", 10, setPositionCallback);
    //set_goalVelocity_sub = n.subscribe("velocity/set", 10, setGoalVelocityCallback);
    //set_torque_sub = n.subscribe("torque/enable", 10, setTorqueEnableCallback);
    //set_led_sub = n.subscribe("led/set", 10, setLedCallback);
    //set_operatingMode_sub = n.subscribe("mode/set", 10, setOperatingModeCallback);
    //set_homingOffset_sub = n.subscribe("homingOffset/set", 10, setHomingOffsetCallback);

    return;
}