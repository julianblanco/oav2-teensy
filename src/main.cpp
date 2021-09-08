/*
 * objee
 */

#include <FreeRTOS_TEENSY4.h>
#include "config.h"
#include "imu.h"
#include "flightcontroller.h"




ACTUATORS g_actuators;
ATTITUDE g_attiude;
IMU g_imu;
NAVIGATION g_navigation;
SENSORS g_sensor;
TELEMETRY g_telemetry;




void setup()
{
  delay(500);

  if (!HITL)
  {
    Serial.begin(57600);
    Serial.println("Start");
    Serial2.begin(57600);
    
    
    int actuators = g_actuators.setup();
    int attitude = g_attitude.setup();
    int imu_start = g_imu.setup();
    int navigation_start = g_navigation.setup();
    int sensor_start = g_sensor.setup();
    int telemetry_start = g_sensor.setup();

  }
  else
  {
    Serial.begin(2000000);
    Serial2.begin(2000000);
    Serial2.println("Start HITL");
    sem = xSemaphoreCreateCounting(1, 0);
    s5 = xTaskCreate(telemetry, NULL, configMINIMAL_STACK_SIZE, NULL, 6, &Handle_telemetry);
    s4 = xTaskCreate(trajectoryControl, NULL, configMINIMAL_STACK_SIZE, NULL, 5, &Handle_navigationTask);
    s3 = xTaskCreate(positionControl, NULL, configMINIMAL_STACK_SIZE, NULL, 4, &Handle_desiredAttitudeTask);
    s2 = xTaskCreate(attitudeControl, NULL, configMINIMAL_STACK_SIZE, NULL, 3, &Handle_attitudeTask);
    s1 = xTaskCreate(actuarorsThread, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &Handle_acutatorsTask);
    s0 = xTaskCreate(readComputer, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &Handle_commsTask);

    // check for creation errors
    if (sem == NULL || s0 != pdPASS || s1 != pdPASS || s2 != pdPASS || s3 != pdPASS || s4 != pdPASS)
    { //|| s5 != pdPASS ) {
      Serial2.println("Creation problem");
      while (1)
        ;
    }
  }

  Serial.println("Starting the scheduler !");

  // start scheduler
  vTaskStartScheduler();
  Serial2.println("Insufficient RAM");
  while (1)
    ;
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{
  // Not used.
}