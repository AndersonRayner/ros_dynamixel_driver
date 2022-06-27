
#include <ros/ros.h>
#include <stdint.h>

#include "dynamixel_msgs/GetHomingOffset.h"
#include "dynamixel_msgs/GetGoalPosition.h"
#include "dynamixel_msgs/GetMoving.h"
#include "dynamixel_msgs/GetMovingStatus.h"
#include "dynamixel_msgs/GetOperatingMode.h"
#include "dynamixel_msgs/GetPresentCurrent.h"
#include "dynamixel_msgs/GetPresentPosition.h"
#include "dynamixel_msgs/GetPresentPwm.h"
#include "dynamixel_msgs/GetPresentVelocity.h"
#include "dynamixel_msgs/GetTorqueEnable.h"

#include "dynamixel_msgs/SetGoalCurrent.h"
#include "dynamixel_msgs/SetHomingOffset.h"
#include "dynamixel_msgs/SetGoalPosition.h"
#include "dynamixel_msgs/SetGoalVelocity.h"
#include "dynamixel_msgs/SetLedEnable.h"
#include "dynamixel_msgs/SetOperatingMode.h"
#include "dynamixel_msgs/SetProfileVelocity.h"
#include "dynamixel_msgs/SetTorqueEnable.h"
#include "dynamixel_msgs/SetVelocityLimit.h"
#include "dynamixel_msgs/SetVelocityPGain.h"
#include "dynamixel_msgs/SetVelocityIGain.h"

// Enums


// Publishers
void init_publishers(ros::NodeHandle n);
void update_publishers();

// Services
void init_services(ros::NodeHandle n);

int32_t getGoalPosition(int32_t *goalPosition);
int32_t getGoalReached(uint8_t *goalReached);
int32_t getPresentCurrent(float *presentCurrent);
int32_t getPresentPosition(int32_t *presentPosition);
int32_t getPresentPwm(float *presentPwm);
int32_t getPresentVelocity(int32_t *presentVelocity);
int32_t getTorqueEnabled(uint8_t *torqueEnabled);
int32_t getOperatingMode(uint8_t *operatingMode);

int32_t setGoalCurrent(uint16_t goalCurrent);
int32_t setGoalVelocity(int32_t speed);
int32_t setGoalPosition(int32_t position);
int32_t setTorqueEnable(uint8_t enable);
int32_t setOperatingMode(uint8_t mode);
int32_t setLED(uint8_t enable);

// Variables
extern uint8_t _servo_id;
