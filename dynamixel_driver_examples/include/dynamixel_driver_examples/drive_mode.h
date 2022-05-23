
#include <ros/ros.h>
#include <stdint.h>

#include "dynamixel_driver/GetHomingOffset.h"
#include "dynamixel_driver/GetGoalPosition.h"
#include "dynamixel_driver/GetMoving.h"
#include "dynamixel_driver/GetMovingStatus.h"
#include "dynamixel_driver/GetOperatingMode.h"
#include "dynamixel_driver/GetPresentCurrent.h"
#include "dynamixel_driver/GetPresentPosition.h"
#include "dynamixel_driver/GetPresentPwm.h"
#include "dynamixel_driver/GetPresentVelocity.h"
#include "dynamixel_driver/GetTorqueEnable.h"

#include "dynamixel_driver/SetGoalCurrent.h"
#include "dynamixel_driver/SetHomingOffset.h"
#include "dynamixel_driver/SetGoalPosition.h"
#include "dynamixel_driver/SetGoalVelocity.h"
#include "dynamixel_driver/SetLedEnable.h"
#include "dynamixel_driver/SetOperatingMode.h"
#include "dynamixel_driver/SetProfileVelocity.h"
#include "dynamixel_driver/SetTorqueEnable.h"
#include "dynamixel_driver/SetVelocityLimit.h"
#include "dynamixel_driver/SetVelocityPGain.h"
#include "dynamixel_driver/SetVelocityIGain.h"

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
