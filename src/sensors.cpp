
/*
 * Record and Push imu for Laphable
*/
#include "sensors.h"
SENSORS::SENSORS()
    :
{
}
SENSORS::~SENSORS() {}

int SENSORS::setup()
{
//
#ifdef MPU6050
  mpu6050init();
#endif
#ifdef BNO055
//shit
#endif
  Task::setup("sensors", 1);
}

int SENSORS::start()
{
  while (1)
  {
    gpsSample(gpsobject);
    vTaskDelay((configTICK_RATE_HZ) / 1000L);
  }
}

void gpsSample(Adafruit_GPS &gpsobject)
{
  if (gpsobject.newNMEAreceived())
  {
    if (gpsobject.parse(gpsobject.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    {
      g_sensors.GPS_fix = 1;
      g_sensors.new_GPS_data = 1;
    }
  }
}
