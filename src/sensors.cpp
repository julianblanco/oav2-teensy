
/*
 * Record and Push imu for Laphable
*/
#ifndef _sensors_H_
#define _sesnors_H_
#endif

#include gps.h
void gpsSample(Adafruit_GPS &gpsobject)
{
  int Fix = 0;
  int new_GPS_data = 0;
  float GpsSpeed = 0;
  float Gpsheading = 0;
  float GpsAltitude = 0;
  float GpsSat = 0;
  float Hour = 0;
  float Minute = 0;
  float Seconds = 0;
  if (gpsobject.newNMEAreceived())
  {
    if (gpsobject.parse(gpsobject.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    {
      Fix = 1;
      new_GPS_data = 1;
      currentLat = gpsobject.latitudeDegrees;
      currentLong = gpsobject.longitudeDegrees;
      GpsSpeed = gpsobject.speed;
      Gpsheading = gpsobject.angle;
      GpsAltitude = gpsobject.altitude;
      GpsSat = gpsobject.satellites;
      Hour = gpsobject.hour;
      Minute = gpsobject.minute;
      Seconds = gpsobject.seconds;
    }
  }
}
