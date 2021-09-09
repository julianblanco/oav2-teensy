#include "actuators.h"

ACTUATORS::ACTUATORS()
    :
{
}
ACTUATORS::~ACTUATORS() {}

int ACTUATORS::setup()
{
  Task::setup("actuators", 1);
}

int ACTUATORS::start()
{
  while (1)
  { 
    if(armed) update_motors();
    else stop_motors();
    vTaskDelay((configTICK_RATE_HZ) / 1000L);
  }
}
void sendHITLmotorcommands()
{
  Serial.print(g_actuators.frontRightMotorSignal);
  Serial.print(',');
  Serial.print(g_actuators.frontLeftMotorSignal);
  Serial.print(',');
  Serial.print(g_actuators.backRightMotorSignal);
  Serial.print(',');
  Serial.println(g_actuators.backLeftMotorSignal);
  // Serial2.println("test");
}

void update_motors()
{
  if (vechicle_type == 0) //quadcopter
  {
    g_actuators.frontRightMotor.writeMicroseconds(g_actuators.frontRightMotorSignal);
    g_actuators.frontLeftMotor.writeMicroseconds(g_actuators.frontLeftMotorSignal);
    g_actuators.backRightMotor.writeMicroseconds(g_actuators.backRightMotorSignal);
    g_actuators.backLeftMotor.writeMicroseconds(g_actuators.backLeftMotorSignal);
  }
  if (vechicle_type == 1) //plane
  {
    g_actuators.yawMotor.writeMicroseconds(g_actuators.yawSignal);
    g_actuators.throttle.writeMicroseconds(g_actuators.throttleSignal);
  }
}

void stop_motors()
{
  if (vechicle_type == 0) //quadcopter
  {
    g_actuators.frontRightMotor.writeMicroseconds(1000);
    g_actuators.frontLeftMotor.writeMicroseconds(1000);
    g_actuators.backRightMotor.writeMicroseconds(1000);
    g_actuators.backLeftMotor.writeMicroseconds(1000);
  }
  if (vechicle_type == 1) //plane
  {
    g_actuators.yawMotor.writeMicroseconds(1500);
    g_actuators.throttle.writeMicroseconds(1500);
  }
}