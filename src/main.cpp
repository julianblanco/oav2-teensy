/*
 * Example to demonstrate thread definition, semaphores, and thread sleep.
 */

#include <FreeRTOS_TEENSY4.h>
#include "config.h"
#include "imu.h"
#include "flightcontroller.h"
// Declare a semaphore handle.
SemaphoreHandle_t sem;


u_int32_t time1 = 0;
u_int32_t time2 = 0;
u_int32_t timebetweenparses = 0;



IMU g_imu;


void setup()
{
  // digitalWrite(LED_PIN, LOW);
  portBASE_TYPE s0, s1, s2, s3, s4, s5;

  delay(500);

  if (!HITL)
  {
    Serial.begin(57600);
    Serial.println("Start");
    Serial2.begin(57600);
    init_motors();
    init_radios();
    init_imu();
    
    // initialize semaphore
    sem = xSemaphoreCreateCounting(1, 0);
    s5 = xTaskCreate(telemetry, NULL, configMINIMAL_STACK_SIZE, NULL, 6, &Handle_telemetry);
    s4 = xTaskCreate(IMU::imuLoop, NULL, configMINIMAL_STACK_SIZE, &imu, 5, &imu.task);
    s3 = xTaskCreate(trajectoryControl, NULL, configMINIMAL_STACK_SIZE, NULL, 4, &Handle_desiredAttitudeTask);
    s2 = xTaskCreate(positionControl, NULL, configMINIMAL_STACK_SIZE, NULL, 3, &Handle_navigationTask);
    s1 = xTaskCreate(attitudeControl, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &Handle_attitudeTask);
    s0 = xTaskCreate(actuarorsThread, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &Handle_acutatorsTask);

    // check for creation errors
    if (sem == NULL || s0 != pdPASS || s1 != pdPASS || s2 != pdPASS)
    {
      Serial.println("Creation problem");
      while (1)
        ;
    }
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