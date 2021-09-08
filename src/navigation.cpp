//*****************************************************************
// Navigation Thread 1HZ
//*****************************************************************
#include "navigation.h"
NAVIGATION::NAVIGATION()
    :
{
}
NAVIGATION::~NAVIGATION() {}

int NAVIGATION::setup()
{
//
#ifdef MPU6050
  mpu6050init();
#endif
#ifdef BNO055
//shit
#endif
  Task::setup("navigation", 1);
}

int NAVIGATION::start()
{
  while (1)
  {

     vTaskDelay((configTICK_RATE_HZ) / 1000L);
  }
}

static void NEDpositionControl(void *pvParameters)
{
  float northError = desiredNorth - currentNorth;
  float eastError = currentEast - desiredEast;
  float downError = currentDown - desiredDown;

  float northResponse = 0;
  float eastResponse = 0;
  float downResponse = 0;

  float pNorthResponse = 0;
  float pEastResponse = 0;
  float pDownResponse = 0;

  float iNorthResponse = 0;
  float iEastResponse = 0;
  float iDownResponse = 0;

  float dNorthResponse = 0;
  float dEastResponse = 0;
  float dDownResponse = 0;

  float northErrorLast = 0;
  float eastErrorLast = 0;
  float downErrorLast = 0;
  float mixer = 0;
  while (1)
  {
    if (flag_armed)
    {
      if (currentMode == 2)
      {
        northError = currentNorth - desiredNorth;
        eastError = currentEast - desiredEast;
        downError = currentDown - desiredDown;

        pNorthResponse = northError * northKp;
        pEastResponse = eastError * eastKp;
        pDownResponse = downError * downKp;

        iNorthResponse += northError * northKi;
        iEastResponse += eastError * eastKi;
        iDownResponse += downError * downKi;

        dNorthResponse = northKd * (northError - northErrorLast) / 10;
        dEastResponse = eastKd * (eastError - eastErrorLast) / 10;
        dDownResponse = downKd * (downError - downErrorLast) / 10;

        northResponse = pNorthResponse + iNorthResponse + dNorthResponse;
        eastResponse = pEastResponse + iEastResponse + dEastResponse;
        downResponse = pDownResponse + iDownResponse + dDownResponse;

        desiredPitch = saturate(northResponse, 30, -30);
        desiredRoll = saturate(eastResponse, 30, -30);
        desiredThrottle = saturate(downResponse, 1000, -1000);
      }
      if (currentMode == 1)
      {
        //handled elsewhere
      }
    }

    //
  }
static void headingFromGPS(void *pvParameters)
{
  int waypointNumber = 0;
  int numOfWaypoints = 5; //size of waypoints
  float oldHeading = 0;
  float courseBetweenWaypoints = 0;
  float distanceBetweenWaypoints = 0;
  float targetHeading = 0;
  float lastdesiredLat = 0;
  float lastdesiredLong = 0;
  while (1)
  {
    //calculates the distance to the waypoint
    float distanceToTarget = distanceToWaypoint(currentLat, currentLong, desiredLat, desiredLong);

    if (flag_are_waypointing == 1)
    {
      float XTerror = crossTrackError(distanceToTarget, courseBetweenWaypoints, oldHeading);
      if (waypointNumber > 1 && XTerror > 1)
      {
        oldHeading = courseToWaypoint(currentLat, currentLong, desiredLat, desiredLong);
        targetHeading = crossTrackCorrection(XTerror, oldHeading, distanceToTarget);
      }
      else
      {
        targetHeading = courseToWaypoint(currentLat, currentLong, desiredLat, desiredLong);
      }

      if (waypointNumber > 1)
      {
        lastdesiredLat = wplat[waypointNumber - 1];
        lastdesiredLong = wplong[waypointNumber - 1];
      }

      if (distanceToTarget < waypointmindistance)
      {
        waypointNumber = waypointNumber + 1;

        if (waypointNumber == numOfWaypoints)
        {
          waypointNumber = 1;
        }
      }

      desiredLat = wplat[waypointNumber];
      desiredLong = wplong[waypointNumber];

      distanceBetweenWaypoints = distanceToWaypoint(desiredLat, desiredLong, wplat[waypointNumber - 1], wplong[waypointNumber - 1]);
      courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber - 1], wplong[waypointNumber - 1], desiredLat, desiredLong);
    }
  }


  float crossTrackCorrection(float distanceXT, float targetHead, float distance2WP)
{
  float xtCoeff = -100; // based on experimental data from the autonomous car
  float temp = (xtCoeff * distanceXT) / distance2WP;

  if (temp > 30)
    temp = 30; // maximum allowable correction
  if (temp < -30)
    temp = -30;

  float newTargetHeading = targetHead + temp;

  if (newTargetHeading >= 360)
    newTargetHeading -= 360;
  else if (newTargetHeading < 0)
    newTargetHeading += 360;

  return newTargetHeading;
} // end crossTrackError


float crossTrackError(float distance2WP, float tracklegHead, float targetHead)
{
  //convert to radians for use with sin
  tracklegHead = (3.14159265 / 180) * tracklegHead;
  targetHead = (3.14159265 / 180) * targetHead;

  //compute heading error off trackline
  float deltaHeading = tracklegHead - targetHead;

  // crosstrack distance (positive if right of track)
  float distanceXT = distance2WP * sin(deltaHeading);

  return distanceXT;
}

//helper functions s
//**************************************************************************
float correct_heading_wrap(float current_heading)
{
  // Correct 360 deg wrap around
  if (current_heading >= 360)
    current_heading -= 360;
  else if (current_heading < 0)
    current_heading += 360;

  return (current_heading);
}

float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2)
{
  float dist;
  float dLat = (float)(Lat2 - Lat1);               // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(Long2 - Long1) * cos(Lat1); //
  dist = sqrt(sq(dLat) + sq(dLon)) * 110312;

  return dist;
}

float courseToWaypoint(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
} // courseToWaypoint()

void convertToNED(float startLat, float startLong, float currentLat, float currentLong, float &North, float &East, float &Down)
{
  float heading = courseToWaypoint(startLat, startLong, currentLat, currentLong);
  float distance = distanceToWaypoint(startLat, startLong, currentLat, currentLong);
  North = distance * sin(heading);
  East = distance * cos(heading);
  // Down = currentRangeSensorHeight - heightOffset;
}

