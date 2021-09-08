
/*
 * Record and Push imu for Laphable
*/
#ifndef _navigation_H_
#define _navigation_H_
#include "task.h"
class NAVIGATION : public Task
{
  // Public interface methods
public:
  NAVIGATION();
  ~NAVIGATION();
#define waypointmindistance 2

float currentNorth = 0;
float currentEast = 0;
float currentDown = 0;

float desiredNorth = 0;
float desiredEast = 0;
float desiredDown = -1;

float currentLat = 0;
float currentLong = 0;

float wplat[] = {
    39.292186,
    39.292130,
    39.291964,
    39.291709,
    39.292016};
float wplong[] = {
    -77.286096,
    -77.285766,
    -77.286192,
    -77.286213,
    -77.285894};

float desiredLat = 0;
float desiredLong = 0;

float homeLat = 0;
float homeLong = 0;
};
#endif