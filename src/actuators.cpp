#include "actuators.h"
 #include "config.h"
ACTUATORS::ACTUATORS() {}
ACTUATORS::~ACTUATORS(){}

int ACTUATORS::setup()
{
  g_actuators.frontLeftMotor.attach(3);
  g_actuators.frontRightMotor.attach(4);
  g_actuators.backLeftMotor.attach(5);
  g_actuators.backRightMotor.attach(6);

  g_actuators.frontLeftMotor.writeMicroseconds(1000);
  g_actuators.frontRightMotor.writeMicroseconds(1000);
  g_actuators.backLeftMotor.writeMicroseconds(1000);
  g_actuators.backRightMotor.writeMicroseconds(1000);
  delay(3000);

  g_actuators.yawMotor.writeMicroseconds(1500);
  g_actuators.throttle.writeMicroseconds(1500);

  // this->log("[+] initialized audio controller\n");

  return 0;
  Task::setup("actuators", 1);
}

int ACTUATORS::start()
{
  while (1)
  {
#ifdef HITL
    sendHITLmotorcommands();

#else

    if (g_armed)
    {
      update_motors();
    }
    else
    {
      stop_motors();
    }
#endif
    vTaskDelay((configTICK_RATE_HZ) / 1000L);
  }
}
void ACTUATORS::sendHITLmotorcommands()
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

void ACTUATORS::update_motors()
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

void ACTUATORS::stop_motors()
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
