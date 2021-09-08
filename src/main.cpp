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





ACTUATORS g_actuators;
ATTITUDE g_attiude;
IMU g_imu;
NAVIGATION g_navigation;
SENSORS g_sensors;
TELEMETRY g_telemetry;




void main()
{
  delay(500);

  
    Serial.begin(57600);
    Serial.println("Start");
    Serial2.begin(57600);
    
    
    int actuators = g_actuators.setup();
    int attitude = g_attitude.setup();
    int imu_start = g_imu.setup();
    int navigation_start = g_navigation.setup();
    int sensor_start = g_sensor.setup();
    int telemetry_start = g_sensor.setup();
  vTaskStartScheduler();
 
  while (1)
    ;
}
