/**
 * objeeairlines, flightcontroller for quadrotors based on the teensy4 and freertos
 */

#include <FreeRTOS_TEENSY4.h>
#include "config.h"

#include "actuators.h"
#include "attitude.h"
#include "imu.h"
#include "navigation.h"
#include "sensors.h"
#include "telemetry.h"

int g_armed = 0;
int g_current_mode = 0;
int g_vehicle_type = 0;
int highPWMmotor = 1500;
int lowPWMmotor = 1000;
int throttleIdle = 1080;

ACTUATORS g_actuators;
ATTITUDE g_attitude;
IMU g_imu;
NAVIGATION g_navigation;
SENSORS g_sensors;
TELEMETRY g_telemetry;

int main()
{
  int flag = 0;
  flag += g_telemetry.setup();
  if (flag)
  {
    Serial.println("error starting telemtry thread");
    while (1)
      ;
  }
  Serial.println("telemetry started");
  flag += g_actuators.setup();
  if (flag)
  {
    Serial.println("error starting a thread");
    while (1)
      ;
  }
  flag += g_attitude.setup();
  if (flag)
  {
    Serial.println("error starting a thread");
    while (1)
      ;
  }
  flag += g_imu.setup();
  if (flag)
  {
    Serial.println("error starting a thread");
    while (1)
      ;
  }
  flag += g_navigation.setup();
  if (flag)
  {
    Serial.println("error starting a thread");
    while (1)
      ;
  }
  flag += g_sensors.setup();
  if (flag)
  {
    Serial.println("error starting a thread");
    while (1)
      ;
  }

  vTaskStartScheduler();

  while (1)
    ;

  return 0;
}
