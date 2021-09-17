
#ifndef _config_H_
#define _config_H_

/*!
 * 
 *
 *  @mainpage objeeAirlines version 2
 *
 *  @section intro_sec Introduction
 *
 *  This code is designed as a lightweight flightcontroller to fly small UAS.
 *
 *  Designed specifically to run the freeRTOS real time operating system on a teensy 4\n
 *  It supports multiple flight modes and craft types.\n\n 
 *  For control of the aircraft, hover envelope and geometric controller are options.
 * 
 *  @section
 * 
 *  By default the code intializes by spawning six threads:  \n
 * 
 *  - actuators
 *  - attitude
 *  - imu
 *  - slow sensors
 *  - navigation
 *  - telemetry
 * 
 *  Each thread is responsible for a diffrent high level tasks. Each high level task, has it own class that stores data
 *  related to said task.\n An object of that class is instatiated as a global variable in main to allow for data to accessed
 *  easily while maitaining code readability.\n RWLocks are used to prevent misreads/miswrites of data. \n\n configuration vaules are declared in config.h and set in main \n\n More information is availble in the documentation for each class.
 * 
 *  @section author Authors
 *
 *  Patrick Ledzian\n
 *  Julian Blanco
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "telemetry.h"
#include "imu.h"
#include "navigation.h"
#include "actuators.h"
#include "attitude.h"
#include "sensors.h"
#include <Arduino.h>
// #include "Eigen/Eigen.h"
// #inc
#include "ArduinoMacroFix.hpp"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Dense>


  #define LOOPFREQ(frequency) vTaskDelay((configTICK_RATE_HZ) / (frequency));
extern bool vechicle_type; //0quadcopter,1plane,2car
// #define HITL
// #define gps_enable
// #define lidar_enable

// #define mpu6050
// #define bno055
#define BNO080

// #define SDLogging
#define mavlinkserial Serial2
#define sbsuserial Serial1
#define dshotserial Serial3
//#define pwm
#define dshot
extern int lowPWMmotor;
extern int highPWMmotor;

extern int g_current_mode;
extern int g_armed;
extern int throttleIdle;
extern int g_vehicle_type;
extern TELEMETRY g_telemetry;
extern ATTITUDE g_attitude;
extern ACTUATORS g_actuators;
extern IMU g_imu;
extern NAVIGATION g_navigation;
extern SENSORS g_sensors;
#endif