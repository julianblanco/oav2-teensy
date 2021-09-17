

/*
 * Attitude
*/
#ifndef _attitude_H_
#define _attitude_H_
#include "task.h"
#include "config.h"


class ATTITUDE : public Task
{
  // Public interface methods
public:
  ATTITUDE();
  ~ATTITUDE();
  float yaw;
  float roll;
  float pitch;
  cpp_freertos::ReadWriteLockPreferWriter lock;

  /**
   * Setup the imu
   *
   * This method will initialize all needed IMUs including, but not
   * limited to:
   * - BNO055
   * - BNO080
   * - MPU6050
   * - 
   */
  int setup();
  int start();
  void PIDAttitudeControl();
 void  basic_attitude_controller();
private:
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

  float yawKp = 0.1;
  float yawKi = 0;
  float yawKd = 0;

  float rollKp = 4;
  float rollKi = 0;
  float rollKd = 4;

  float pitchKp = 4;
  float pitchKi = 0;
  float pitchKd = 4;

  float throtKp = 10;
  float throtKi = 0;
  float throtKd = 0;

// Setpoints for step response trajectory
bool _set_pt1 = 1;
bool _set_pt2 = 0;
bool _set_pt3 = 0;
bool _set_pt4 = 0;

// Minimum snap trajectory variables
Eigen::MatrixXd _traj_setpoints(6, 3);      // positions to achieve
Eigen::MatrixXd _ts(5,1);                   // one less x dimension as _trajectory
Eigen::MatrixXd _coef(40, 3);               // 8*(m_-1) x 3; where m_ is the row dimension of _traj_setpoints; resized in optimizer
double _total_traj_time;
bool _is_optimized = 0;                           // 0 -- not yet optimized, 1 -- optimal coefficients found
bool _start_traj = 0;                             // 0 -- not started,       1 -- started / in-progress
double _traj_time = 0.0;                          // current trajectory time
double _traj_start_time = 0.0;
bool _traj_finished = 0;

// Measured sensor values
Eigen::Matrix<double,1,4> _sensor_quat((Eigen::Matrix<double,1,4>() << 1.0, 0.0, 0.0, 0.0).finished());
Eigen::Matrix<double,1,3> _sensor_pos;

// Previous values for derivations
Eigen::Matrix<double,1,3> _prev_sensor_pos;
Eigen::Matrix<double,1,3> _prev_derived_euler_att;

// Derived values from sensor measurments
Eigen::Matrix<double,1,3> _derived_lin_vel;
Eigen::Matrix<double,1,3> _derived_euler_att;
Eigen::Matrix<double,1,3> _derived_pqr_att;

// Desired values to achieve
Eigen::Matrix<double,1,3> _desired_pos;
Eigen::Matrix<double,1,3> _desired_vel;
Eigen::Matrix<double,1,3> _desired_acc;
Eigen::Matrix<double,1,3> _desired_euler_att;
Eigen::Matrix<double,1,3> _orig_desired_euler_att;          // for attitude troubleshooting
Eigen::Array<double,1,3> _desired_pqr_att;
Eigen::Matrix<double,1,4> _desired_thrust((Eigen::Matrix<double,1,4>() << 0.0, 0.0, 0.0, 0.0).finished());
double _desired_tot_thrust_delta;

// Helper variables
Eigen::Matrix3d _q_hat((Eigen::Matrix3d() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
Eigen::Matrix<double,1,4> _quat_normalized((Eigen::Matrix<double,1,4>() << 1.0, 0.0, 0.0, 0.0).finished());
Eigen::Matrix<double,1,3> _final_att_deltas;

// Controller gains
static Eigen::Matrix<double,1,3> _Kp_pos;
static Eigen::Matrix<double,1,3> _Kd_pos;
static Eigen::Matrix<double,1,3> _Kp_ang;
static Eigen::Matrix<double,1,3> _Kd_ang;

};
// extern ATTITUDE g_attitude;
#endif