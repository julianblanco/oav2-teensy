#include "actuators.h"
void sendHITLmotorcommands()
{
      Serial.print(frontRightMotorSignal);
      Serial.print(',');
      Serial.print(frontLeftMotorSignal);
      Serial.print(',');
      Serial.print(backRightMotorSignal);
      Serial.print(',');
      Serial.println(backLeftMotorSignal);
      // Serial2.println("test");
}

void update_motors(){
        if (vechicle_type == 1)
        {
          yawMotor.writeMicroseconds(yawSignal);
          throttle.writeMicroseconds(throttleSignal);
        }
        if (vechicle_type == 0)
        {
          frontRightMotor.writeMicroseconds(frontRightMotorSignal);
          frontLeftMotor.writeMicroseconds(frontLeftMotorSignal);
          backRightMotor.writeMicroseconds(backRightMotorSignal);
          backLeftMotor.writeMicroseconds(backLeftMotorSignal);
        }
}

void stop_motors()
{
    if (vechicle_type == 1)
        {
          yawMotor.writeMicroseconds(1500);
          throttle.writeMicroseconds(1500);
        }
        if (vechicle_type == 0)
        {
          frontRightMotor.writeMicroseconds(1000);
          frontLeftMotor.writeMicroseconds(1000);
          backRightMotor.writeMicroseconds(1000);
          backLeftMotor.writeMicroseconds(1000);
        }
}