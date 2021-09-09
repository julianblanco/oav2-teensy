/*
 * Record and Push imu for Laphable
*/
#ifndef _sensors_H_
#define _sensors_H_
#include "task.h"
#include "Adafruit_GPS.h"
#include <Arduino.h>
#include "helper.h"
#define gpsobject Serial2

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
Adafruit_GPS GPS(gpsobject);
void gpsSample(Adafruit_GPS &gpsobject);
float currentRangeSensorHeight = 0;
float heightOffset = 0;
};
extern SENSORS g_sensors;
#endif
