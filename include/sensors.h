/*
 * Record and Push imu for Laphable
*/
#ifndef _sensors_H_
#define _sensors_H_

#include <Adafruit_GPS.h>
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
void gpsSample(Adafruit_GPS &gpsobject);
float currentRangeSensorHeight = 0;
float heightOffset = 0;


#endif
