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
  Task::setup("attitude", 8); //name of thread
  return 0;
}

int ATTITUDE::start()
{
  while (1)
  {
    //getdata
    ATTITUDE::PIDAttitudeControl();
    LOOPFREQ(400); //hz
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
  g_imu.lock.ReaderLock();
  _derived_euler_att << g_imu.roll,g_imu.pitch,g_imu.yaw;
  g_imu.lock.ReaderUnlock();

  Eigen::Array3d att_deltas_;

  // calculate attitude deltas for mapping to motor thrusts
  att_deltas_(0) = 1.0 * (_Kp_ang(0) * (_desired_euler_att(0) - _derived_euler_att(0))) + 1.0 * (_Kd_ang(0) * (_desired_pqr_att(0) - _derived_pqr_att(0)));
  att_deltas_(1) = 1.0 * (_Kp_ang(1) * (_desired_euler_att(1) - _derived_euler_att(1))) + 1.0 * (_Kd_ang(1) * (_desired_pqr_att(1) - _derived_pqr_att(1)));
  att_deltas_(2) = 1.0 * (_Kp_ang(2) * (_desired_euler_att(2) - _derived_euler_att(2))) + 1.0 * (_Kd_ang(2) * (_desired_pqr_att(2) - _derived_pqr_att(2)));

  _final_att_deltas = att_deltas_; // for logging purposes

  // 4x1 vector to be multiplied by the thrust mapping matrix
  Eigen::Matrix<double, 4, 1> all_deltas_;
  all_deltas_ << (_hover_point + _desired_tot_thrust_delta), att_deltas_(0), att_deltas_(1), att_deltas_(2);

  _desired_thrust = (_motor_mapping * all_deltas_); // derive desired rotor rates

} // end basic_attitude_controller()

Eigen::Matrix<double, 1, 3> ATTITUDE::quat2euler(const Eigen::Ref<const Eigen::Matrix<double, 1, 4>> &q_)
{
  // References
  // previously listed papers
  // Nice approximation of atan2: https://www.dsprelated.com/showarticle/1052.php
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
  //

  Eigen::Matrix<double, 3, 3> rotation_;
  Eigen::Matrix<double, 1, 3> euler_;
  euler_ << 0.0, 0.0, 0.0;

  rotation_ = quat2rot(q_);
  euler_(0) = asin(rotation_(1, 2)); // roll
  euler_(1) = atan2((-1.0 * rotation_(0, 2)) / cos(euler_(0)),
                    (rotation_(2, 2) / cos(euler_(0))));
  euler_(2) = atan2((-1.0 * rotation_(1, 0)) / cos(euler_(0)),
                    (rotation_(1, 1) / cos(euler_(0))));

  return (euler_);

} // end Quadcopter::quat2euler()
// Convert quaternions to a rotation matrix -- used in quat2euler()
Eigen::Matrix<double, 3, 3> ATTITUDE::quat2rot(const Eigen::Ref<const Eigen::Matrix<double, 1, 4>> &q_)
{
  Eigen::Matrix<double, 3, 3> q_hat_;
  Eigen::Matrix<double, 1, 4> quat_normalized_;

  quat_normalized_ = q_.normalized();
  q_hat_(0, 0) = 0.0;
  q_hat_(0, 1) = -1.0 * quat_normalized_(3);
  q_hat_(0, 2) = quat_normalized_(2);
  q_hat_(1, 0) = quat_normalized_(3);
  q_hat_(1, 1) = 0.0;
  q_hat_(1, 2) = -1.0 * quat_normalized_(1);
  q_hat_(2, 0) = -1.0 * quat_normalized_(2);
  q_hat_(2, 1) = quat_normalized_(1);
  q_hat_(2, 2) = 0.0;

  return (Eigen::Matrix<double, 3, 3>::Identity() + (2.0 * q_hat_ * q_hat_) + (2.0 * quat_normalized_(0) * q_hat_)); // returns a rotation matrix

} // end Quadcopter::quat2rot()

//// Derives angular velocity vector from euler angles
Eigen::Matrix<double, 1, 3> ATTITUDE::derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double, 1, 3>> &e_, const Eigen::Ref<const Eigen::Matrix<double, 1, 3>> &prev_e_)
{
  // References:
  // The GRASP Multiple Micro UAV Testbed
  //

  Eigen::Matrix<double, 3, 3> tfm_; // transformation matrix

  tfm_(0, 0) = cos(e_(1));
  tfm_(0, 1) = 0.0;
  tfm_(0, 2) = (-1.0 * cos(e_(0)) * sin(e_(1)));
  tfm_(1, 0) = 0.0;
  tfm_(1, 1) = 1.0;
  tfm_(1, 2) = sin(e_(0));
  tfm_(2, 0) = sin(e_(1));
  tfm_(2, 1) = 0.0;
  tfm_(2, 2) = (cos(e_(0)) * cos(e_(1)));

  return (tfm_ * (e_ - prev_e_).transpose()); // angular velocity vector

} // end Quadcopter::derive_ang_velocity()