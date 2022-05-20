
#include "stdint.h"


// Control Table of ROM Area
static const uint16_t ADDR_MODEL_NUMBER = 0; //	2 	Model Number 	R 	1,200 	- 	-
static const uint32_t ADDR_MODEL_INFO = 2; //	4 	Model Information 	R 	- 	- 	-
static const uint8_t  ADDR_FIRMWARE_VERSION = 6; //	1 	Firmware Version 	R 	- 	- 	-
static const uint8_t  ADDR_ID = 7; //	1 	ID 	RW 	1 	0 ~ 252 	-
static const uint8_t  ADDR_BAUDRATE = 8; //	1 	Baud Rate 	RW 	1 	0 ~ 6 	-
static const uint8_t  ADDR_RETURN_DELAY_TIME = 9; //	1 	Return Delay Time 	RW 	250 	0 ~ 254 	2 [μsec ]
static const uint8_t  ADDR_DRIVE_MODE = 10; //	1 	Drive Mode 	RW 	0 	0 ~ 5 	-
static const uint8_t  ADDR_OPERATING_MODE = 11; //	1 	Operating Mode 	RW 	3 	0 ~ 16 	-
static const uint8_t  ADDR_SECONDARY_ID = 12; //	1 	Secondary(Shadow) ID 	RW 	255 	0 ~ 252 	-
static const uint8_t  ADDR_PROTOCOL_TYPE = 13; //	1 	Protocol Type 	RW 	2 	2 ~ 22 	-
static const uint32_t ADDR_HOMING_OFFSET = 20; 	// 4 	Homing Offset 	RW 	0 	-1,044,479 ~ 1,044,479 	1 [pulse ]
static const uint32_t ADDR_MOVING_THRESHOLD = 24;  //	4 	Moving Threshold 	RW 	10 	0 ~ 1,023 	0.229 [rev/min]
static const uint8_t  ADDR_TEMPERATURE_LIMIT = 31;  //	1 	Temperature Limit 	RW 	70 	0 ~ 100 	1 [°C]
static const uint16_t ADDR_MAX_VOLTAGE_LIMIT = 32; //	2 	Max Voltage Limit 	RW 	70 	31 ~ 70 	0.1 [V]
static const uint16_t ADDR_MIN_VOLTAGE_LIMIT = 34; //	2 	Min Voltage Limit 	RW 	35 	31 ~ 70 	0.1 [V]
static const uint16_t ADDR_PWM_LIMIT = 36; //	2 	PWM Limit 	RW 	885 	0 ~ 885 	0.113 [%]
static const uint16_t ADDR_CURRENT_LIMIT = 38; //	2 	Current Limit 	RW 	1,750 	0 ~ 1,750 	1 [mA ]
static const uint32_t ADDR_VELOCITY_LIMIT = 44; //	4 	Velocity Limit 	RW 	445 	0 ~ 2,047 	0.229 [rev/min]
static const uint32_t ADDR_MAX_POSITION_LIMIT = 48; //	4 	Max Position Limit 	RW 	4,095 	0 ~ 4,095 	1 [pulse]
static const uint32_t ADDR_MIN_POSITION_LIMIT = 52; //	4 	Min Position Limit 	RW 	0 	0 ~ 4,095 	1 [pulse]
static const uint8_t  ADDR_STARTUP_CONFIGURATION = 60; //	1 	Startup Configuration 	RW 	0 	3 	-
static const uint8_t  ADDR_PWM_SLOPE = 62; //	1 	PWM Slope 	RW 	140 	1 ~ 255 	1.977 [mV/msec]
static const uint8_t  ADDR_SHUTDOWN = 63; // 	1 	Shutdown 	RW 	53 	- 	-

// Control Table of RAM Area
static const uint8_t  ADDR_TORQUE_ENABLE    =  64; // 1 	Torque Enable 	RW 	0 	0 ~ 1 	-
static const uint8_t  ADDR_LED_ENABLE  = 65; //	1 	LED 	RW 	0 	0 ~ 1 	-
static const uint8_t  ADDR_STATUS_RETURN_LEVEL = 68; // 	1 	Status Return Level 	RW 	2 	0 ~ 2 	-
static const uint8_t  ADDR_REGISTERED_INSTRUCTION = 69; // 	1 	Registered Instruction 	R 	0 	0 ~ 1 	-
static const uint8_t  ADDR_HARDWARE_ERROR_STATUS = 70; // 	1 	Hardware Error Status 	R 	0 	- 	-
static const uint16_t ADDR_VELOCITY_I_GAIN = 76; // 	2 	Velocity I Gain 	RW 	1,600 	0 ~ 16,383 	-
static const uint16_t ADDR_VELOCITY_P_GAIN = 78; // 	2 	Velocity P Gain 	RW 	180 	0 ~ 16,383 	-
static const uint16_t ADDR_POSITION_D_GAIN = 80; // 	2 	Position D Gain 	RW 	0 	0 ~ 16,383 	-
static const uint16_t ADDR_POSTION_I_GAIN = 82; // 	2 	Position I Gain 	RW 	0 	0 ~ 16,383 	-
static const uint16_t ADDR_POSTION_P_GAIN = 84; // 	2 	Position P Gain 	RW 	400 	0 ~ 16,383 	-
static const uint16_t ADDR_FEEDFORWARD_2ND_GAIN = 88; // 	2 	Feedforward 2nd Gain 	RW 	0 	0 ~ 16,383 	-
static const uint16_t ADDR_FEEDFORWARD_1ST_GAIN = 90; // 	2 	Feedforward 1st Gain 	RW 	0 	0 ~ 16,383 	-
static const uint8_t  ADDR_BUS_WATCHDOG = 98; // 	1 	Bus Watchdog 	RW 	0 	1 ~ 127 	20 [msec]
static const uint16_t ADDR_GOAL_PWM = 100; // 	2 	Goal PWM 	RW 	- 	-PWM Limit(36) ~ PWM Limit(36) 	-
static const uint16_t ADDR_GOAL_CURRENT = 102; // 	2 	Goal Current 	RW 	- 	-Current Limit(38) ~ Current Limit(38) 	1 [mA]
static const uint32_t ADDR_GOAL_VELOCITY = 104; // 	4 	Goal Velocity 	RW 	- 	-Velocity Limit(44) ~ Velocity Limit(44) 	0.229 [rev/min]
static const uint32_t ADDR_PROFILE_ACCELERATION = 108; // 	4 	Profile Acceleration 	RW 	0 	0 ~ 32,767 0 ~ 32,737 	214.577 [rev/min2] 1 [ms]
static const uint32_t ADDR_PROFILE_VELOCITY = 112; // 	4 	Profile Velocity 	RW 	0 	0 ~ 32,767 	0.229 [rev/min]
static const uint32_t ADDR_GOAL_POSITION    = 116; // 4 Goal Position	RW 	- 	Min Position Limit(52) ~ Max Position Limit(48) 	1 [pulse]
static const uint16_t ADDR_REALTIME_TICK = 120; // 	2 	Realtime Tick 	R 	- 	0 ~ 32,767 	1 [msec]
static const uint8_t  ADDR_MOVING = 122; // 	1 	Moving 	R 	0 	0 ~ 1 	-
static const uint8_t  ADDR_MOVING_STATUS = 123; // 	1 	Moving Status 	R 	0 	- 	-
static const uint16_t ADDR_PRESENT_PWM = 124; // 	2 	Present PWM 	R 	- 	- 	-
static const uint16_t ADDR_PRESENT_CURRENT = 126; // 	2 	Present Current 	R 	- 	- 	1 [mA]
static const uint32_t ADDR_PRESENT_VELOCITY = 128; // 	4 	Present Velocity 	R 	- 	- 	0.229 [rev/min]
static const uint32_t ADDR_PRESENT_POSITION = 132; // 	4 	Present Position 	R 	- 	- 	1 [pulse]
static const uint32_t ADDR_VELOCITY_TRAJECTORY = 136; // 	4 	Velocity Trajectory 	R 	- 	- 	0.229 [rev/min]
static const uint32_t ADDR_POSITION_TRAJECTORY = 140; // 	4 	Position Trajectory 	R 	- 	- 	1 [pulse]
static const uint16_t ADDR_PRESENT_INPUT_VOLTAGE = 144; // 	2 	Present Input Voltage 	R 	- 	- 	0.1 [V]
static const uint8_t  ADDR_PRESENT_TEMPERATURE = 146; // 	1 	Present Temperature 	R 	- 	- 	1 [°C]
static const uint8_t  ADDR_BACKUP_READY = 147; // 	1 	Backup Ready 	R 	- 	0 ~ 1
