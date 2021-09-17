#include "attitude.h"
 #include "config.h"
ATTITUDE::ATTITUDE() {}

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
  Task::setup("attitude",8);
  return 0;
}

int ATTITUDE::start()
{
  while (1)
  {
    //getdata
    ATTITUDE::PIDAttitudeControl();
    LOOPFREQ(400);//hz
  }
}
//*****************************************************************
// attitudeControl Sensor 100hz
//*****************************************************************
void ATTITUDE::PIDAttitudeControl()
{

  if (g_armed)
  {

    rollErrorLast = rollError;
    pitchErrorLast = pitchError;
    yawErrorLast = yawError;
    g_imu.lock.ReaderLock();
    yawError = angular_diff(g_imu.yaw, g_navigation.desiredYaw);
    rollError = g_imu.roll - g_navigation.desiredRoll;
    pitchError = g_imu.pitch - g_navigation.desiredPitch;
    g_imu.lock.ReaderUnlock();

    pYawResponse = yawError * yawKp;
    pRollResponse = rollError * rollKp;
    pPitchResponse = pitchError * pitchKp;
    pThrotResponse = g_navigation.desiredThrottle;

    iYawResponse += yawError * yawKi;
    iRollResponse += rollError * rollKi;
    iPitchResponse += pitchError * pitchKi;

    dYawResponse = yawKd * (yawError - yawErrorLast) / 100; //10microseconds
    dRollResponse = rollKd * (rollError - rollErrorLast) / 100;
    dPitchResponse = pitchKd * (pitchError - pitchErrorLast) / 100;

    yawResponse = pYawResponse + iYawResponse - dYawResponse;
    rollResponse = pRollResponse + iRollResponse - dRollResponse;
    pitchResponse = pPitchResponse + iPitchResponse - dPitchResponse;

    throtResponse = g_navigation.desiredThrottle;

    if (g_vehicle_type == 0)
    {
      if (throtResponse < 75)
      {
        g_actuators.frontLeftMotorSignal = 1000;
        g_actuators.frontRightMotorSignal = 1000;
        g_actuators.backLeftMotorSignal = 1000;
        g_actuators.backRightMotorSignal = 1000;
      }
      else
      {
        // Serial.print("hello");
        g_actuators.frontLeftMotorSignal = (int)(throttleIdle + throtResponse - rollResponse + pitchResponse - yawResponse);
        g_actuators.frontRightMotorSignal = (int)(throttleIdle + throtResponse + rollResponse + pitchResponse + yawResponse);
        g_actuators.backLeftMotorSignal = (int)(throttleIdle + throtResponse - rollResponse - pitchResponse + yawResponse);
        g_actuators.backRightMotorSignal = (int)(throttleIdle + throtResponse + rollResponse - pitchResponse - yawResponse);
      }

      // Serial.println(backRightMotorSignal);
    }
    if (g_vehicle_type == 1)
    {
      g_actuators.yawSignal = 1500 + yawResponse;
      g_actuators.throttleSignal = 1500 + throtResponse;
    }

    g_actuators.yawSignal = saturate(g_actuators.yawSignal, highPWMmotor, 1000);
    g_actuators.throttleSignal = saturate(g_actuators.throttleSignal, highPWMmotor, 1000);
    g_actuators.backRightMotorSignal = saturate(g_actuators.backRightMotorSignal, highPWMmotor, 1000);
    g_actuators.backLeftMotorSignal = saturate(g_actuators.backLeftMotorSignal, highPWMmotor, 1000);
    g_actuators.frontRightMotorSignal = saturate(g_actuators.frontRightMotorSignal, highPWMmotor, 1000);
    g_actuators.frontLeftMotorSignal = saturate(g_actuators.frontLeftMotorSignal, highPWMmotor, 1000);
    // Serial.println(backRightMotorSignal);
  }
  else
  {
    g_actuators.yawSignal = 1500;
    g_actuators.throttleSignal = 1500;
    g_actuators.backRightMotorSignal = 1000;
    g_actuators.backLeftMotorSignal = 1000;
    g_actuators.frontRightMotorSignal = 1000;
    g_actuators.frontLeftMotorSignal = 1000;
  }
}


//// Attitude controller; uses error feedback (approximates SO(3))
// same reference papers as above
void ATTITUDE::basic_attitude_controller()
{
    // quaternion is being normalized by Gazebo
    _derived_euler_att = quat2euler(_sensor_quat);

    Eigen::Array3d att_deltas_;

    // calculate attitude deltas for mapping to motor thrusts
    att_deltas_(0) = 1.0*(_Kp_ang(0) * (_desired_euler_att(0) - _derived_euler_att(0)))
                            + 1.0*(_Kd_ang(0) * (_desired_pqr_att(0) - _derived_pqr_att(0)));
    att_deltas_(1) = 1.0*(_Kp_ang(1) * (_desired_euler_att(1) - _derived_euler_att(1)))
                            + 1.0*(_Kd_ang(1) * (_desired_pqr_att(1) - _derived_pqr_att(1)));
    att_deltas_(2) = 1.0*(_Kp_ang(2) * (_desired_euler_att(2) - _derived_euler_att(2)))
                            + 1.0*(_Kd_ang(2) * (_desired_pqr_att(2) - _derived_pqr_att(2)));

    _final_att_deltas = att_deltas_;        // for logging purposes

    // 4x1 vector to be multiplied by the thrust mapping matrix
    Eigen::Matrix<double,4,1> all_deltas_;
    all_deltas_ << (_hover_point + _desired_tot_thrust_delta), att_deltas_(0), att_deltas_(1), att_deltas_(2);

    _desired_thrust =  (_motor_mapping * all_deltas_);          // derive desired rotor rates

} // end basic_attitude_controller()
