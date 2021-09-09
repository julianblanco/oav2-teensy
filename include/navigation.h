
/*
 * Record and Push imu for Laphable
*/
#ifndef _navigation_H_
#define _navigation_H_
#include "task.h"
// #include "config.h"
class NAVIGATION : public Task
{
  // Public interface methods
public:
  NAVIGATION();
  ~NAVIGATION();
  int start();
  int setup();
  void headingFromGPS();
  void NEDpositionControl();
  void quickdebug();
  float  crossTrackCorrection(float distanceXT, float targetHead, float distance2WP);
float crossTrackError(float distance2WP, float tracklegHead, float targetHead);
float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2);
float courseToWaypoint(float lat1, float long1, float lat2, float long2);
void convertToNED(float startLat, float startLong, float currentLat, float currentLong, float &North, float &East, float &Down);
#define waypointmindistance 2
int flag_are_waypointing;
float currentNorth ;
float currentEast ;
float currentDown ;

float desiredNorth ;
float desiredEast ;
float desiredDown;

float currentLat ;
float currentLong ;

float desiredYaw ;
float desiredRoll ;
float desiredPitch ;
float desiredThrottle ;


  float northKp ;
  float northKi ;
  float northKd ;

  float eastKp ;
  float eastKi;
  float eastKd;

  float downKp;
  float downKi ;
  float downKd;


float wplat[] ;
float wplong[] ;

float desiredLat ;
float desiredLong ;

float homeLat ;
float homeLong ;
};
// extern NAVIGATION g_navigation;
#endif