
/*
 * Record and Push imu for Laphable
*/
#ifndef _actuators_H_
#define _actuators_H_

#include <servo.h>
Servo yawMotor;
Servo throttle;

Servo frontLeftMotor;
Servo frontRightMotor;
Servo backLeftMotor;
Servo backRightMotor;
int yawSignal = 1500;
int throttleSignal = 1500;
int frontLeftMotorSignal = 1000;
int frontRightMotorSignal = 1000;
int backLeftMotorSignal = 1000;
int backRightMotorSignal = 1000;

#endif