
#ifndef _config_H_
#define _config_H_

#include "telemetry.h"
#include "imu.h"
#include "navigation.h"
#include "actuators.h"
#include "attitude.h"
#include "sensors.h"
#include <Arduino.h>

extern bool vechicle_type; //0quadcopter,1plane,2car
// #define HITL
#define gps_enable
// #define lidar_enable

#define mpu6050
// #define bno055
// #define bno080

#define SDLogging
#define mavlinkserial Serial2
#define sbsuserial Serial1
extern int lowPWMmotor;
extern int highPWMmotor;

extern int g_current_mode;
extern int g_armed;
extern int throttleIdle;
extern TELEMETRY g_telemetry;
extern ATTITUDE g_attitude;
extern ACTUATORS g_actuators;
extern IMU g_imu;
extern NAVIGATION g_navigation;
extern SENSORS g_sensors;
#endif