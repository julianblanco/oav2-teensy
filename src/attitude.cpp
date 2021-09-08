#include "attitude.h"
#include "imu.h"
ATTITUDE::ATTITUDE()
    :
{
}
ATTITUDE::~ATTITUDE() {}

int ATTITUDE::setup()
{
//
#ifdef MPU6050
  mpu6050init();
#endif
#ifdef BNO055
//shit
#endif
  Task::setup("attitude", 1);
}

int ATTITUDE::start()
{
  while (1)
  {
    //getdata
    vTaskDelay((configTICK_RATE_HZ) / 1000L);
  }
}
//*****************************************************************
// attitudeControl Sensor 100hz
//*****************************************************************
static void PIDAttitudeControl()
{

  float yawResponse = 0;
  float rollResponse = 0;
  float pitchResponse = 0;
  float throtResponse = 0;

  float yawError = 0;
  float rollError = 0;
  float pitchError = 0;
  float throtError = 0;

  float pYawResponse = 0;
  float pRollResponse = 0;
  float pPitchResponse = 0;
  float pThrotResponse = 0;

  float iYawResponse = 0;
  float iRollResponse = 0;
  float iPitchResponse = 0;
  float iThrotResponse = 0;

  float dYawResponse = 0;
  float dRollResponse = 0;
  float dPitchResponse = 0;
  float dThrotResponse = 0;

  float yawErrorLast = 0;
  float rollErrorLast = 0;
  float pitchErrorLast = 0;
  float throtErrorLast = 0;
  while (1)
  {
    if (flag_armed)
    {

      rollErrorLast = rollError;
      pitchErrorLast = pitchError;
      yawErrorLast = yawError;
      g_imu.lock.ReaderLock();
      yawError = angular_diff(g_imu.yaw, desiredYaw);
      rollError = g_imu.roll - desiredRoll;
      pitchError = g_imu.pitch - desiredPitch;
      g_imu.lock.ReaderUnlock();

      pYawResponse = yawError * yawKp;
      pRollResponse = rollError * rollKp;
      pPitchResponse = pitchError * pitchKp;
      pThrotResponse = desiredThrottle;

      iYawResponse += yawError * yawKi;
      iRollResponse += rollError * rollKi;
      iPitchResponse += pitchError * pitchKi;

      dYawResponse = yawKd * (yawError - yawErrorLast) / 100; //10microseconds
      dRollResponse = rollKd * (rollError - rollErrorLast) / 100;
      dPitchResponse = pitchKd * (pitchError - pitchErrorLast) / 100;

      yawResponse = pYawResponse + iYawResponse - dYawResponse;
      rollResponse = pRollResponse + iRollResponse - dRollResponse;
      pitchResponse = pPitchResponse + iPitchResponse - dPitchResponse;

      throtResponse = desiredThrottle;
      if (!HITL)
      {
        if (vechicle_type == 0)
        {
          if (throtResponse < 75)
          {
            frontLeftMotorSignal = 1000;
            frontRightMotorSignal = 1000;
            backLeftMotorSignal = 1000;
            backRightMotorSignal = 1000;
          }
          else
          {
            frontLeftMotorSignal = (int)(throttleIdle + throtResponse - rollResponse + pitchResponse - yawResponse);
            frontRightMotorSignal = (int)(throttleIdle + throtResponse + rollResponse + pitchResponse + yawResponse);
            backLeftMotorSignal = (int)(throttleIdle + throtResponse - rollResponse - pitchResponse + yawResponse);
            backRightMotorSignal = (int)(throttleIdle + throtResponse + rollResponse - pitchResponse - yawResponse);
          }

          // Serial.println(backRightMotorSignal);
        }
        if (vechicle_type == 1)
        {
          yawSignal = 1500 + yawResponse;
          throttleSignal = 1500 + throtResponse;
        }

        yawSignal = saturate(yawSignal, highPWMmotor, 1000);
        throttleSignal = saturate(throttleSignal, highPWMmotor, 1000);
        backRightMotorSignal = saturate(backRightMotorSignal, highPWMmotor, 1000);
        backLeftMotorSignal = saturate(backLeftMotorSignal, highPWMmotor, 1000);
        frontRightMotorSignal = saturate(frontRightMotorSignal, highPWMmotor, 1000);
        frontLeftMotorSignal = saturate(frontLeftMotorSignal, highPWMmotor, 1000);
        // Serial.println(backRightMotorSignal);
      }
      else
      {
        backRightMotorSignal = HITLthrottleIdle + throtResponse - rollResponse + pitchResponse + yawResponse;
        backLeftMotorSignal = HITLthrottleIdle + throtResponse + rollResponse + pitchResponse - yawResponse;
        frontRightMotorSignal = HITLthrottleIdle + throtResponse - rollResponse - pitchResponse - yawResponse;
        frontLeftMotorSignal = HITLthrottleIdle + throtResponse + rollResponse - pitchResponse + yawResponse;

        backRightMotorSignal = saturate(backRightMotorSignal, 600, 600);
        backLeftMotorSignal = saturate(backLeftMotorSignal, 600, 600);
        frontRightMotorSignal = saturate(frontRightMotorSignal, 600, 600);
        frontLeftMotorSignal = saturate(frontLeftMotorSignal, 600, 600);
      }
    }
    else
    {
      yawSignal = 1500;
      throttleSignal = 1500;
      backRightMotorSignal = 1000;
      backLeftMotorSignal = 1000;
      frontRightMotorSignal = 1000;
      frontLeftMotorSignal = 1000;
    }
  }
}