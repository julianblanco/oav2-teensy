/*
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

int g_flag_armed = 0;
int g_current_mode = 0;
ACTUATORS g_actuators;
ATTITUDE g_attitude;
IMU g_imu;
NAVIGATION g_navigation;
SENSORS g_sensors;
TELEMETRY g_telemetry;

int main()
{
  int flag = 0;
  flag += g_actuators.setup();
  flag += g_attitude.setup();
  flag += g_imu.setup();
  flag += g_navigation.setup();
  flag += g_sensors.setup();
  flag += g_telemetry.setup();
  if (flag == 0) 
  {
    Serial.println("error starting a thread");
    while(1);
  }
  vTaskStartScheduler();

  while (1)
    ;
    
    return 0;
}
