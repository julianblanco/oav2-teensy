
/*
 * Record and Push imu for Laphable
*/
#ifndef _navigation_H_
#define _navigation_H_
#include "task.h"
#include "ArduinoMacroFix.hpp"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <vector>
// #include "config.h"
class NAVIGATION : public Task
{
  // Public interface methods
public:
  NAVIGATION();
  ~NAVIGATION();
  int start();
  int setup();
  void headingFromGPS();
  void NEDpositionControl();
  void quickdebug();
  float  crossTrackCorrection(float distanceXT, float targetHead, float distance2WP);
float crossTrackError(float distance2WP, float tracklegHead, float targetHead);
float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2);
float courseToWaypoint(float lat1, float long1, float lat2, float long2);
void convertToNED(float startLat, float startLong, float currentLat, float currentLong, float &North, float &East, float &Down);
void minimum_snap_trajectory();
void generate_ts();
void  min_snap_optimization();
void figure_eight_trajectory();
void circling_trajectory();
void basic_position_controller();
#define waypointmindistance 2

Eigen::Matrix<double,1,4> _sensor_quat;
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
Eigen::Matrix<double,1,4> _desired_thrust();


// Controller gains
static Eigen::Matrix<double,1,3> _Kp_pos;
static Eigen::Matrix<double,1,3> _Kd_pos;
static Eigen::Matrix<double,1,3> _Kp_ang;
static Eigen::Matrix<double,1,3> _Kd_ang;

// environmental variables
double _gravity = 9.81;     // m/s^2

// initialize physical properties of the vehicle
double _mass = 1.5;          // kg
double _hover_point = 665.0; // rad/s for one motor only
double _motor_force_const = 8.54858e-06;
double _desired_tot_thrust_delta;

int flag_are_waypointing;
float currentNorth ;
float currentEast ;
float currentDown ;

float desiredNorth ;
float desiredEast ;
float desiredDown;

float currentLat ;
float currentLong ;

float desiredYaw ;
float desiredRoll ;
float desiredPitch ;
float desiredThrottle ;


  float northKp ;
  float northKi ;
  float northKd ;

  float eastKp ;
  float eastKi;
  float eastKd;

  float downKp;
  float downKi ;
  float downKd;


float wplat[] ;
float wplong[] ;

float desiredLat ;
float desiredLong ;

float homeLat ;
float homeLong ;

// simulation time
double sim_time = 0.0;
double prev_sim_time = 0.0;
double prev_sim_time_pos = 0.0;
double prev_sim_time_att = 0.0;
double sim_time_delta = 0.0;

double _total_traj_time;
bool _is_optimized = 0;                           // 0 -- not yet optimized, 1 -- optimal coefficients found
bool _start_traj = 0;                             // 0 -- not started,       1 -- started / in-progress
double _traj_time = 0.0;                          // current trajectory time
double _traj_start_time = 0.0;
bool _traj_finished = 0;

// Minimum snap trajectory variables
Eigen::MatrixXd _traj_setpoints;      // positions to achieve
Eigen::MatrixXd _ts;                   // one less x dimension as _trajectory
Eigen::MatrixXd _coef;               // 8*(m_-1) x 3; where m_ is the row dimension of _traj_setpoints; resized in optimizer


};

#endif