/*
 * Record and Push imu for Laphable
*/
#ifndef _sensors_H_
#define _sensors_H_
#include "task.h"
#include "Adafruit_GPS.h"
// #include <Arduino.h>
#include "HardwareSerial.h"
#include "helper.h"
// #include "config.h"
#define gpsserial Serial2

class SENSORS : public Task
{
  // Public interface methods
public:
  SENSORS();
  ~SENSORS();
  int setup();
  int start();
  int GPS_fix;
  bool new_GPS_data;
  Adafruit_GPS gps;
  void gpsSample();
  float currentRangeSensorHeight ;
  float heightOffset;
  
};
// extern SENSORS g_sensors;
#endif
