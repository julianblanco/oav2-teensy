
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

  Task::setup("sensors", 1);
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
    vTaskDelay((configTICK_RATE_HZ) / 1000L);
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
