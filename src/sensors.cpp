
/*
 * Slow sensors code
*/
#include "sensors.h"
 #include "config.h"
 #include "HardwareSerial.h"
SENSORS::SENSORS():gps(&gpsserial)
{
}
SENSORS::~SENSORS() {}

int SENSORS::setup()
{

  Task::setup("sensors", 5);
}

int SENSORS::start()
{
  while (1)
  {
    #ifdef gps_enable
    gpsSample();
    #endif
    #ifdef lidar_enable
    #endif
    LOOPFREQ(100);//hz
  }
}

void SENSORS::gpsSample()
{
  if (gps.newNMEAreceived())
  {
    if (gps.parse(gps.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    {
      GPS_fix = 1;
      new_GPS_data = 1;
    }
  }
}
